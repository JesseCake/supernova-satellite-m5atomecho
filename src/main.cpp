/*
 * Supernova satellite — M5 Atom Echo firmware
 * ─────────────────────────────────────────────────────────────────────────────
 * C++/PlatformIO port of satellite_client.py for the M5 Atom Echo
 * (ESP32-PICO-D4, PDM mic, I2S speaker, one button, one SK6812 LED).
 *
 * Protocol (same as the Pi client, plus one new frame):
 *
 *   Client → Server:
 *     HELO  JSON {"id": ..., "name": ...}  once per (re)connect
 *     WAKE  optional payload = CALL announcement text being echoed back
 *     MIC1  no payload — marks start of fresh listening window
 *     AUD0  int16 mono 16 kHz PCM chunk
 *     BYE0  no payload — user hang-up (NEW; server should end the session
 *           and reply CLOS, exactly like its normal session-end path)
 *
 *   Server → Client:
 *     RDY0  ready for audio
 *     TTS0  int16 mono 16 kHz PCM (streamed to I2S, never fully buffered)
 *     BEEP  int16 mono 16 kHz PCM
 *     THNK  server thinking
 *     CLOS  session over, connection persists
 *     CALL  server-initiated session, optional announcement text payload
 *
 * Button — straight swap for the wake word, plus hang-up:
 *     press while IDLE       → send WAKE (start a session)
 *     press during a session → send BYE0 (hang up). Playback is cut
 *                              immediately; further TTS0 is discarded until
 *                              CLOS arrives. If the server doesn't answer
 *                              with CLOS within HANGUP_FALLBACK_MS (e.g. the
 *                              BYE0 handler isn't implemented yet), the
 *                              satellite force-closes the session locally.
 *
 * Differences from the Pi client, forced by hardware:
 *   • No wake word — the button plays the role of the "_WAKE" event.
 *   • Audio payloads are streamed from the socket in 2 KB chunks directly
 *     into the I2S DMA (no PSRAM; a single utterance wouldn't fit in heap).
 *   • Mic and speaker share GPIO33, so the I2S driver is reinstalled when
 *     switching direction. The protocol is half-duplex anyway, so this is
 *     invisible at the session level.
 *
 * LED legend:
 *   blinking yellow  Wi-Fi connecting     dim white   IDLE (connected)
 *   blinking red     reconnect backoff    yellow      WAITING
 *   blue             LISTENING            pulsing purple  THINKING
 *   green            SPEAKING             orange      CLOSING / hanging up
 */

#include <Arduino.h>
#include <WiFi.h>
#include <FastLED.h>
#include <driver/i2s.h>
#include "config.h"

// ── Pins (fixed by the Atom Echo board) ──────────────────────────────────────
static const int PIN_BUTTON   = 39;
static const int PIN_LED      = 27;
static const int PIN_SPK_BCLK = 19;
static const int PIN_I2S_WS   = 33;   // speaker LRCK *and* mic PDM clock
static const int PIN_SPK_DOUT = 22;
static const int PIN_MIC_DATA = 23;

// ── Audio constants (match the Pi client) ────────────────────────────────────
static const uint32_t SAMPLE_RATE  = 16000;
static const size_t   MIC_CHUNK    = 512;    // samples per AUD0 frame (32 ms)
static const size_t   STREAM_CHUNK = 2048;   // bytes per TTS0 socket read

// ── Frame protocol: 4-byte tag + uint32 LE length ────────────────────────────
struct FrameHeader {
  char     tag[4];
  uint32_t length;      // little-endian on the wire; ESP32 is LE, so memcpy ok
} __attribute__((packed));

static bool tagIs(const char t[4], const char *s) { return memcmp(t, s, 4) == 0; }

// ── State machine (mirrors satellite_client.State) ───────────────────────────
enum class State { IDLE, WAITING, LISTENING, THINKING, SPEAKING, CLOSING };
static const char *stateName(State s) {
  switch (s) {
    case State::IDLE:      return "IDLE";
    case State::WAITING:   return "WAITING";
    case State::LISTENING: return "LISTENING";
    case State::THINKING:  return "THINKING";
    case State::SPEAKING:  return "SPEAKING";
    case State::CLOSING:   return "CLOSING";
  }
  return "?";
}

// ── Globals ──────────────────────────────────────────────────────────────────
static WiFiClient client;
static State      state          = State::IDLE;
static bool       micStreaming   = false;   // AUD0 frames flowing
static bool       sessionActive  = false;
static bool       hangingUp      = false;   // BYE0 sent, waiting for CLOS
static bool       discardTTS     = false;   // swallow TTS0 until CLOS
static uint32_t   hangupSentMs   = 0;
static uint32_t   lastEventMs    = 0;
static uint32_t   lastSessionEnd = 0;       // for WAKE_COOLDOWN_MS
static float      reconnectDelay = 2000.0f; // exponential backoff, ms

// Pending WAKE payload (CALL announcement echo, like _pending_call_payload)
static char   pendingCallPayload[256];
static size_t pendingCallLen = 0;

static CRGB led[1];

// ═════════════════════════════════════════════════════════════════════════════
// LED
// ═════════════════════════════════════════════════════════════════════════════

static void ledShow(CRGB c) { led[0] = c; FastLED.show(); }

static void ledUpdate() {
  uint32_t t = millis();
  if (!client.connected()) { ledShow((t / 400) % 2 ? CRGB::Red : CRGB::Black); return; }
  if (hangingUp)           { ledShow(CRGB(255, 100, 0)); return; }
  switch (state) {
    case State::IDLE:      ledShow(CRGB(40, 40, 40)); break;
    case State::WAITING:   ledShow(CRGB::Yellow);     break;
    case State::LISTENING: ledShow(CRGB::Blue);       break;
    case State::THINKING: {
      uint8_t v = beatsin8(45, 20, 255);              // gentle pulse
      ledShow(CRGB(v / 2, 0, v));
      break;
    }
    case State::SPEAKING:  ledShow(CRGB::Green);        break;
    case State::CLOSING:   ledShow(CRGB(255, 100, 0));  break;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// I2S — half-duplex, reinstalled on direction change (GPIO33 is shared)
// ═════════════════════════════════════════════════════════════════════════════

enum class I2SMode { NONE, SPEAKER, MIC };
static I2SMode i2sMode = I2SMode::NONE;

static void i2sSetMode(I2SMode mode) {
  if (mode == i2sMode) return;
  if (i2sMode != I2SMode::NONE) i2s_driver_uninstall(I2S_NUM_0);

  i2s_config_t cfg = {};
  cfg.sample_rate          = SAMPLE_RATE;
  cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format       = I2S_CHANNEL_FMT_ALL_RIGHT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;

  if (mode == I2SMode::MIC) {
    cfg.mode          = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    cfg.dma_buf_count = 8;
    cfg.dma_buf_len   = 256;
  } else {
    cfg.mode               = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.dma_buf_count      = 8;
    cfg.dma_buf_len        = 256;
    cfg.tx_desc_auto_clear = true;
  }

  i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);

  i2s_pin_config_t pins = {};
  pins.bck_io_num   = PIN_SPK_BCLK;
  pins.ws_io_num    = PIN_I2S_WS;
  pins.data_out_num = (mode == I2SMode::SPEAKER) ? PIN_SPK_DOUT : I2S_PIN_NO_CHANGE;
  pins.data_in_num  = (mode == I2SMode::MIC)     ? PIN_MIC_DATA : I2S_PIN_NO_CHANGE;
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

  i2sMode = mode;
}

static void spkWrite(const int16_t *samples, size_t n) {
  size_t written = 0;
  i2s_write(I2S_NUM_0, samples, n * sizeof(int16_t), &written, portMAX_DELAY);
}

// Scale a PCM buffer in place by SPK_VOLUME.
static void applyVolume(int16_t *samples, size_t n) {
  for (size_t i = 0; i < n; i++)
    samples[i] = (int16_t)((int32_t)(samples[i] * SPK_VOLUME));
}

// Rough wait for the I2S TX DMA to drain (dma_buf_count * dma_buf_len samples).
static void spkDrain() { delay((8 * 256 * 1000) / SAMPLE_RATE + 20); }

// ═════════════════════════════════════════════════════════════════════════════
// Local sounds (ports of AudioIO.beep / inbound_alert)
// ═════════════════════════════════════════════════════════════════════════════

static void beep(float freq, float duration, float volume) {
  i2sSetMode(I2SMode::SPEAKER);
  const size_t total = (size_t)(SAMPLE_RATE * duration);
  static int16_t buf[512];
  size_t done = 0;
  const size_t fade = (size_t)(0.004f * SAMPLE_RATE);
  while (done < total) {
    size_t n = min((size_t)512, total - done);
    for (size_t i = 0; i < n; i++) {
      size_t idx = done + i;
      float  v   = sinf(2.0f * PI * freq * idx / SAMPLE_RATE) * volume;
      // 4 ms fade in/out to avoid clicks
      if (idx < fade)          v *= (float)idx / fade;
      if (idx > total - fade)  v *= (float)(total - idx) / fade;
      buf[i] = (int16_t)(constrain(v, -1.0f, 1.0f) * 32767.0f);
    }
    spkWrite(buf, n);
    done += n;
  }
}

static void playSilence(float duration) {
  i2sSetMode(I2SMode::SPEAKER);
  static int16_t zeros[512] = {0};
  size_t total = (size_t)(SAMPLE_RATE * duration);
  while (total) {
    size_t n = min((size_t)512, total);
    spkWrite(zeros, n);
    total -= n;
  }
}

// XDR/SDR cassette tone ladder — the server-initiated CALL alert,
// ported verbatim from AudioIO.inbound_alert().
static void inboundAlert(int repeats = 2) {
  static const float freqs[] = {50, 100, 250, 400, 640, 1010, 1610, 4000, 6350, 8100};
  const int   nFreqs   = sizeof(freqs) / sizeof(freqs[0]);
  const float volume   = 0.45f;
  const float burstLen = 0.18f;
  const float gapLen   = 0.023f;

  for (int rep = 0; rep < repeats; rep++) {
    for (int i = 0; i < nFreqs; i++) {
      float vol = min(volume + ((float)i / nFreqs) * 0.12f, 1.0f);
      beep(freqs[i], burstLen, vol);
      playSilence(gapLen);
    }
    if (rep < repeats - 1) playSilence(0.3f);
  }
}

static void closingBeeps() {
  for (int i = 0; i < 3; i++) { beep(300, 0.20f, 0.6f); delay(150); }
}

// ═════════════════════════════════════════════════════════════════════════════
// Networking primitives
// ═════════════════════════════════════════════════════════════════════════════

static bool sendFrame(const char *tag, const uint8_t *payload = nullptr, uint32_t len = 0) {
  if (!client.connected()) return false;
  FrameHeader hdr;
  memcpy(hdr.tag, tag, 4);
  hdr.length = len;                       // ESP32 is little-endian, matches "<4sI"
  if (client.write((uint8_t *)&hdr, sizeof(hdr)) != sizeof(hdr)) return false;
  if (len && client.write(payload, len) != len) return false;
  return true;
}

// Blocking read with deadline. Returns false on timeout/disconnect.
static bool readExactly(uint8_t *dst, size_t n, uint32_t timeoutMs = 10000) {
  uint32_t deadline = millis() + timeoutMs;
  size_t   got      = 0;
  while (got < n) {
    if (!client.connected()) return false;
    int avail = client.available();
    if (avail > 0) {
      int r = client.read(dst + got, min((size_t)avail, n - got));
      if (r > 0) { got += r; continue; }
    }
    if (millis() > deadline) return false;
    delay(1);
  }
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// Button — debounced press detection (the wake-word stand-in)
// ═════════════════════════════════════════════════════════════════════════════

static bool buttonPressed() {
  static bool     wasDown      = false;
  static uint32_t lastChangeMs = 0;

  bool down = (digitalRead(PIN_BUTTON) == LOW);
  uint32_t now = millis();
  if (now - lastChangeMs < DEBOUNCE_MS) return false;

  if (down != wasDown) {
    wasDown      = down;
    lastChangeMs = now;
    if (down) return true;      // fire on press, not release — feels snappier
  }
  return false;
}

// ═════════════════════════════════════════════════════════════════════════════
// State transitions (mirrors _set_state / _on_state_enter)
// ═════════════════════════════════════════════════════════════════════════════

static void setState(State s) {
  if (s == state) return;
  Serial.printf("[state] %s -> %s\n", stateName(state), stateName(s));
  state = s;
  ledUpdate();

  switch (s) {
    case State::IDLE:
      micStreaming = false;
      break;

    case State::WAITING:
      micStreaming = false;
      beep(1000, 0.08f, 0.3f);
      break;

    case State::LISTENING:
      // handled by enterListening() (needs beep-before-mic-switch ordering)
      break;

    case State::THINKING:
      micStreaming = false;
      // 150 ms of silence primes the amp out of idle so the beep isn't
      // clipped — same trick as the Pi client, and the NS4168 also swallows
      // the first few ms after the I2S driver is (re)installed.
      playSilence(0.15f);
      beep(1000, 0.12f, 0.3f);
      break;

    case State::SPEAKING:
      micStreaming = false;
      break;

    case State::CLOSING:
      micStreaming = false;
      spkDrain();
      closingBeeps();
      break;
  }
}

static void enterListening() {
  beep(700, 0.12f, 0.3f);       // speaker mode
  spkDrain();
  i2sSetMode(I2SMode::MIC);
  // Flush a few reads — equivalent of audio.flush_input(CHUNK, 4)
  static int16_t junk[MIC_CHUNK];
  size_t br;
  for (int i = 0; i < 4; i++)
    i2s_read(I2S_NUM_0, junk, sizeof(junk), &br, pdMS_TO_TICKS(100));
  sendFrame("MIC1");
  micStreaming = true;
  setState(State::LISTENING);
}

static void startSession(const uint8_t *callPayload, size_t callLen) {
  sessionActive = true;
  hangingUp     = false;
  discardTTS    = false;
  lastEventMs   = millis();
  setState(State::WAITING);
  sendFrame("WAKE", callPayload, callLen);
}

static void endSessionToIdle() {
  sessionActive  = false;
  hangingUp      = false;
  discardTTS     = false;
  pendingCallLen = 0;
  lastSessionEnd = millis();
  setState(State::IDLE);
  i2sSetMode(I2SMode::SPEAKER);   // ready for instant beeps / greetings
  Serial.println("[satellite] IDLE - press button to start a session");
}

// Hang up: cut audio, send BYE0, then wait for the server's CLOS.
// Called from the main loop and from mid-frame inside streamAudioPayload.
static void hangUp() {
  if (!sessionActive || hangingUp) return;
  Serial.println("[button] hang up -> BYE0");
  micStreaming = false;
  discardTTS   = true;
  hangingUp    = true;
  hangupSentMs = millis();
  if (i2sMode == I2SMode::SPEAKER) i2s_zero_dma_buffer(I2S_NUM_0);
  sendFrame("BYE0");
  ledUpdate();
}

// ═════════════════════════════════════════════════════════════════════════════
// Mic pump — one AUD0 chunk per call (cooperative, keeps loop responsive)
// ═════════════════════════════════════════════════════════════════════════════

static void pumpMic() {
  static int16_t buf[MIC_CHUNK];
  size_t bytesRead = 0;
  if (i2s_read(I2S_NUM_0, buf, sizeof(buf), &bytesRead, pdMS_TO_TICKS(100)) != ESP_OK
      || bytesRead == 0) return;

  size_t n = bytesRead / sizeof(int16_t);
  // Fixed digital gain with clipping (SPM1423 is quiet)
  for (size_t i = 0; i < n; i++) {
    int32_t v = (int32_t)(buf[i] * MIC_GAIN);
    buf[i] = (int16_t)constrain(v, -32768, 32767);
  }
  if (!sendFrame("AUD0", (uint8_t *)buf, n * sizeof(int16_t)))
    Serial.println("[mic] send error");
}

// ═════════════════════════════════════════════════════════════════════════════
// Incoming audio streaming (TTS0 / BEEP payloads)
// ═════════════════════════════════════════════════════════════════════════════

// Reads `length` bytes of PCM from the socket in chunks, feeding I2S directly.
// The button is polled between chunks so a hang-up cuts playback mid-frame:
// we keep consuming bytes (to stay frame-aligned) but stop writing them.
static bool streamAudioPayload(uint32_t length) {
  static uint8_t buf[STREAM_CHUNK];

  if (!discardTTS) {
    i2sSetMode(I2SMode::SPEAKER);
    if (state != State::SPEAKING && sessionActive) setState(State::SPEAKING);
  }

  uint32_t remaining = length;
  while (remaining) {
    uint32_t n = min((uint32_t)STREAM_CHUNK, remaining);
    // ~1 s of audio should never take >15 s to arrive; generous timeout
    if (!readExactly(buf, n, 15000)) return false;
    remaining -= n;

    if (!discardTTS) {
      size_t samples = n / sizeof(int16_t);
      applyVolume((int16_t *)buf, samples);
      spkWrite((int16_t *)buf, samples);

      if (buttonPressed() && sessionActive) hangUp();   // sets discardTTS
    }
    ledUpdate();
  }
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// Frame dispatch — the merged IDLE-phase + session-phase logic of
// _run_connection(), driven one frame at a time.
// ═════════════════════════════════════════════════════════════════════════════

// Skip a payload we don't care about without buffering it.
static bool skipPayload(uint32_t length) {
  static uint8_t buf[256];
  while (length) {
    uint32_t n = min((uint32_t)sizeof(buf), length);
    if (!readExactly(buf, n)) return false;
    length -= n;
  }
  return true;
}

// Returns false on socket death.
static bool pumpSocket() {
  if (client.available() < (int)sizeof(FrameHeader)) return client.connected();

  FrameHeader hdr;
  if (!readExactly((uint8_t *)&hdr, sizeof(hdr))) return false;
  lastEventMs = millis();

  // ── Audio frames: streamed, never buffered ─────────────────────────────────
  if (tagIs(hdr.tag, "TTS0") || tagIs(hdr.tag, "BEEP")) {
    return streamAudioPayload(hdr.length);
  }

  // ── Small control frames ───────────────────────────────────────────────────
  if (tagIs(hdr.tag, "RDY0")) {
    if (hdr.length && !skipPayload(hdr.length)) return false;
    if (hangingUp) {
      // Server re-opened the mic mid-hangup (race with our BYE0) — ignore;
      // the CLOS or our fallback timer will finish the job.
      Serial.println("[recv] RDY0 ignored (hanging up)");
      return true;
    }
    Serial.println("[recv] RDY0");
    spkDrain();                       // let greeting/TTS finish before mic
    if (!sessionActive) {
      // Server greeted us while IDLE then sent RDY0 — the
      // "channel_already_open" path: enter the session without sending WAKE.
      sessionActive = true;
      hangingUp     = false;
      discardTTS    = false;
    }
    enterListening();
    return true;
  }

  if (tagIs(hdr.tag, "THNK")) {
    if (hdr.length && !skipPayload(hdr.length)) return false;
    if (hangingUp) return true;       // no thinking beep while hanging up
    if (state != State::THINKING) {
      Serial.println("[recv] THNK");
      setState(State::THINKING);
    }
    return true;
  }

  if (tagIs(hdr.tag, "CLOS")) {
    if (hdr.length && !skipPayload(hdr.length)) return false;
    Serial.println("[recv] CLOS - session ended, connection persists");
    discardTTS = false;               // let CLOSING's drain+beeps behave normally
    setState(State::CLOSING);
    endSessionToIdle();
    return true;
  }

  if (tagIs(hdr.tag, "CALL")) {
    // Server-initiated session. Payload = optional announcement text,
    // echoed back in the WAKE frame (like _pending_call_payload).
    pendingCallLen = min((size_t)hdr.length, sizeof(pendingCallPayload));
    if (pendingCallLen && !readExactly((uint8_t *)pendingCallPayload, pendingCallLen))
      return false;
    if (hdr.length > pendingCallLen && !skipPayload(hdr.length - pendingCallLen))
      return false;

    if (state == State::IDLE) {
      Serial.printf("[recv] CALL (%u bytes announcement)\n", (unsigned)pendingCallLen);
      if (pendingCallLen) inboundAlert();
      startSession((uint8_t *)pendingCallPayload, pendingCallLen);
      pendingCallLen = 0;
    } else {
      Serial.println("[recv] CALL ignored (session already active)");
    }
    return true;
  }

  // Unknown frame — log and skip payload to stay aligned.
  Serial.printf("[recv] Unknown tag %.4s (%u bytes)\n", hdr.tag, (unsigned)hdr.length);
  return skipPayload(hdr.length);
}

// ═════════════════════════════════════════════════════════════════════════════
// Button dispatch: IDLE → wake, in-session → hang up
// ═════════════════════════════════════════════════════════════════════════════

static void handleButton() {
  if (!buttonPressed()) return;
  if (!client.connected()) return;

  if (state == State::IDLE) {
    if (millis() - lastSessionEnd < WAKE_COOLDOWN_MS) return;   // cooldown
    Serial.println("[button] wake -> starting session");
    startSession(nullptr, 0);
  } else if (sessionActive) {
    hangUp();
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// Connection lifecycle (mirrors _connect / _run_connection / run_forever)
// ═════════════════════════════════════════════════════════════════════════════

static bool connectToServer() {
  Serial.printf("[satellite] Connecting to %s:%d...\n", SERVER_HOST, SERVER_PORT);
  if (!client.connect(SERVER_HOST, SERVER_PORT, 5000)) return false;
  client.setNoDelay(true);

  char helo[160];
  int n = snprintf(helo, sizeof(helo), "{\"id\":\"%s\",\"name\":\"%s\"}",
                   ENDPOINT_ID, FRIENDLY_NAME);
  if (!sendFrame("HELO", (uint8_t *)helo, n)) { client.stop(); return false; }

  Serial.printf("[satellite] Connected and registered as \"%s\"\n", ENDPOINT_ID);
  return true;
}

static void runConnection() {
  endSessionToIdle();

  while (client.connected()) {
    ledUpdate();
    handleButton();

    if (micStreaming && state == State::LISTENING)
      pumpMic();                      // 32 ms per chunk, keeps loop live

    if (!pumpSocket()) break;         // socket died

    // Hang-up fallback: no CLOS after BYE0 (handler missing server-side?)
    if (hangingUp && millis() - hangupSentMs > HANGUP_FALLBACK_MS) {
      Serial.println("[session] No CLOS after BYE0 - closing locally. "
                     "(Add a BYE0 handler to the server for a clean hang-up.)");
      discardTTS = false;
      setState(State::CLOSING);
      endSessionToIdle();
    }

    // Session watchdog — matches the Pi's 60 s queue.get timeout.
    if (sessionActive && millis() - lastEventMs > SESSION_TIMEOUT_MS) {
      Serial.println("[session] Timed out waiting for server event.");
      setState(State::CLOSING);
      endSessionToIdle();
    }

    delay(1);
  }

  Serial.println("[satellite] Connection lost.");
  client.stop();
  micStreaming  = false;
  sessionActive = false;
  hangingUp     = false;
  discardTTS    = false;
  state         = State::IDLE;
}

static void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("[wifi] Connecting to %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    ledShow((millis() / 300) % 2 ? CRGB::Yellow : CRGB::Black);
    delay(50);
  }
  Serial.printf("[wifi] Connected, IP %s\n", WiFi.localIP().toString().c_str());
}

// ═════════════════════════════════════════════════════════════════════════════
// Arduino entry points
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT);         // GPIO39 is input-only, board has pull-up

  FastLED.addLeds<SK6812, PIN_LED, GRB>(led, 1);
  FastLED.setBrightness(LED_BRIGHTNESS);
  ledShow(CRGB::Black);

  Serial.printf("\n[satellite] Starting up (endpoint_id=\"%s\")...\n", ENDPOINT_ID);

  // Startup beeps — same as the Pi client
  i2sSetMode(I2SMode::SPEAKER);
  playSilence(0.1f);                  // wake the amp
  beep(800, 0.12f, 0.3f);
  delay(80);
  beep(400, 0.12f, 0.3f);
}

void loop() {
  ensureWifi();

  if (!connectToServer()) {
    Serial.printf("[satellite] Connection failed. Retrying in %.0fs...\n",
                  reconnectDelay / 1000.0f);
    uint32_t until = millis() + (uint32_t)reconnectDelay;
    while (millis() < until) { ledUpdate(); delay(50); }
    reconnectDelay = min(reconnectDelay * 2, 60000.0f);
    return;
  }
  reconnectDelay = 2000.0f;           // reset backoff on success

  runConnection();

  Serial.printf("[satellite] Disconnected. Reconnecting in %.0fs...\n",
                reconnectDelay / 1000.0f);
  uint32_t until = millis() + (uint32_t)reconnectDelay;
  while (millis() < until) { ledUpdate(); delay(50); }
  reconnectDelay = min(reconnectDelay * 2, 60000.0f);
}