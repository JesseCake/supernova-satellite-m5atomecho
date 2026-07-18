#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Supernova Atom Echo satellite — user configuration
// Edit this file before flashing.
// ─────────────────────────────────────────────────────────────────────────────

// ── Wi-Fi ────────────────────────────────────────────────────────────────────
#define WIFI_SSID       "your-ssid"
#define WIFI_PASS       "your-password"

// ── Supernova voice server ───────────────────────────────────────────────────
#define SERVER_HOST     "192.168.8.200"   // same host/port as voice_server in config.yaml
#define SERVER_PORT     8765

// ── Identity (mirrors the Pi client's HELO registration) ─────────────────────
// endpoint_id: stable identifier for this satellite, used by the server
// registry to route CALL frames. Keep it unique per device.
#define ENDPOINT_ID     "Jesse-mobile1"
#define FRIENDLY_NAME   "Jesse Mobile Wearable Interface"

// ── Audio tuning ─────────────────────────────────────────────────────────────
// The SPM1423 PDM mic is fairly quiet; a fixed digital gain helps ASR.
// (The Pi client's adaptive gain was only used for wake word detection,
// which doesn't exist here — raw-with-fixed-gain is fine for Whisper.)
#define MIC_GAIN        4.0f    // 1.0 = raw. 3–6 is a sensible range.

// Speaker output scale. The NS4168 distorts near full scale on the tiny
// driver, so leave some headroom.
#define SPK_VOLUME      0.70f   // 0.0–1.0

// ── Button ───────────────────────────────────────────────────────────────────
// Press while IDLE  → WAKE (the wake-word replacement)
// Press in session  → BYE0 (hang up; server should respond with CLOS)
#define DEBOUNCE_MS     30

// If the server hasn't sent CLOS this long after our BYE0 (e.g. the BYE0
// handler isn't implemented server-side yet), force-close locally.
#define HANGUP_FALLBACK_MS  5000

// Cooldown after a wake press before another is accepted, mirroring the Pi's
// wake_cooldown_s (0.8 s default). Mostly relevant across very short sessions.
#define WAKE_COOLDOWN_MS 800

// ── Session ──────────────────────────────────────────────────────────────────
#define SESSION_TIMEOUT_MS  60000   // matches the Pi client's 60s event timeout

// ── LED ──────────────────────────────────────────────────────────────────────
#define LED_BRIGHTNESS  40      // 0–255; the SK6812 is very bright