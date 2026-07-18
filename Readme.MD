# Supernova Satellite — M5 Atom Echo

A C++/PlatformIO port of the Supernova Agent remote voice interface (https://github.com/JesseCake/supernova_satellite_linux) for the M5 Atom Echo. Speaks the
same persistent-connection frame protocol as the Raspberry Pi satellite, so it
registers with the server via `HELO` and appears in the endpoint registry like
any other satellite — including receiving server-initiated `CALL` sessions.

## The button

The button is a straight swap for the wake word, plus hang-up:

| Press | State | Action |
|---|---|---|
| Press | IDLE | Send `WAKE` — start a session (like the Pi's `_WAKE` event) |
| Press | any session state | Send `BYE0` — hang up. Playback cut instantly, closing beeps on `CLOS` |

## Setup

1. Edit `include/config.h` — Wi-Fi credentials, server host/port, `ENDPOINT_ID`
   and `FRIENDLY_NAME` (these go in the `HELO` JSON exactly like the Pi client).
2. Flash:

   ```
   pio run -t upload
   pio device monitor
   ```

No wiring — mic, speaker, button, and LED are all built into the Atom Echo.

## LED

| Colour | Meaning |
|---|---|
| Blinking yellow | Joining Wi-Fi |
| Blinking red | Server unreachable, reconnect backoff |
| Dim white | IDLE — connected, ready for button or `CALL` |
| Yellow | WAITING — `WAKE` sent |
| Blue | LISTENING — mic streaming `AUD0` |
| Pulsing purple | THINKING |
| Green | SPEAKING |
| Orange | CLOSING / hanging up |

## Design notes (why it differs internally from the Pi client)

**Streaming playback instead of a queue.** The Atom Echo has no PSRAM and
roughly 300 KB of usable heap; a single TTS utterance can be 100 KB+. `TTS0`
and `BEEP` payloads are therefore read from the socket in 2 KB chunks and fed
straight into the I2S DMA, never fully buffered. The button is polled between
chunks so a hang-up cuts playback mid-frame: the remaining payload bytes are
still consumed (to keep frame alignment) but discarded, and any further `TTS0`
frames are swallowed until `CLOS` arrives.

**Half-duplex I2S.** The PDM mic clock and speaker LRCK share GPIO33, so the
I2S driver is torn down and reinstalled when switching direction. The session
state machine is already half-duplex (mic only during LISTENING), so this
never conflicts with the protocol. Order on entering LISTENING matters:
beep (speaker) → drain → switch to mic → flush 4 reads → `MIC1` → stream.

**Single cooperative loop instead of threads.** The Pi uses four threads; here
one loop alternates between pumping the mic (32 ms chunks) and pumping the
socket. This is safe because the protocol is strictly sequential on one TCP
stream, and it avoids FreeRTOS task/stack overhead on a 520 KB-RAM part.

**Mic gain.** The Pi's adaptive gain existed only to feed openwakeword; ASR
audio was sent raw. The SPM1423 is quiet, so a fixed `MIC_GAIN` (default 4×,
clipped) is applied to `AUD0`. Tune it in `config.h` if transcripts suffer.

**Hang-up races.** If the server sends `RDY0` or `THNK` after our `BYE0` is in
flight, they're ignored while `hangingUp` is set — the `CLOS` (or the 5 s
fallback) wins.

**Large `TTS0` frames.** Streamed fine at any size, but a 15 s per-socket-read
timeout applies inside a frame; if you ever generate multi-minute announcements
as one frame, chunk them server-side or raise the timeout in
`streamAudioPayload()`.

## Frames implemented

Client → Server: `HELO` `WAKE` `MIC1` `AUD0` `BYE0`
Server → Client: `RDY0` `TTS0` `BEEP` `THNK` `CLOS` `CALL`

Includes the `channel_already_open` path (greeting `TTS0` + `RDY0` arriving
while IDLE enters LISTENING without sending `WAKE`), `CALL` payload echo in
the `WAKE` frame, the XDR/SDR cassette-tone inbound alert, the 60 s session
watchdog, the wake cooldown, and the 2 s → 60 s exponential reconnect backoff
with `HELO` re-registration.