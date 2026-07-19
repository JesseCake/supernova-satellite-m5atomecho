/*
 * Supernova satellite — M5 Atom Echo firmware (With WireGuard Roaming Expansion)
 * ─────────────────────────────────────────────────────────────────────────────
 * Protocol remains half-duplex, running matching frame architectures.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WireGuard-ESP32.h>
#include <FastLED.h>
#include <driver/i2s.h>
#include <time.h>
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
  uint32_t length;      
} __attribute__((packed));

static bool tagIs(const char t[4], const char *s) { return memcmp(t, s, 4) == 0; }

// ── State machine ───────────────────────────────────────────────────────────
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
static WiFiClient  client;
static WiFiMulti   wifiMulti;
static WireGuard   wg;
static State       state          = State::IDLE;
static bool        micStreaming   = false;   
static bool        sessionActive  = false;
static bool        hangingUp      = false;   
static bool        discardTTS     = false;   
static uint32_t    hangupSentMs   = 0;
static uint32_t    lastEventMs    = 0;
static uint32_t    lastSessionEnd = 0;       
static float       reconnectDelay = 2000.0f; 

static String      activeServerHost  = "";
static int         activeServerPort  = AgentConfig::SERVER_PORT;
static String      lastConnectedSSID = "";

// Pending WAKE payload
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
      uint8_t v = beatsin8(45, 20, 255);              
      ledShow(CRGB(v / 2, 0, v));
      break;
    }
    case State::SPEAKING:  ledShow(CRGB::Green);        break;
    case State::CLOSING:   ledShow(CRGB(255, 100, 0));  break;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// I2S — half-duplex management
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

static void applyVolume(int16_t *samples, size_t n) {
  for (size_t i = 0; i < n; i++)
    samples[i] = (int16_t)((int32_t)(samples[i] * SPK_VOLUME));
}

static void spkDrain() { delay((8 * 256 * 1000) / SAMPLE_RATE + 20); }

// ═════════════════════════════════════════════════════════════════════════════
// Local sounds 
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
// Networking & Environment Switching
// ═════════════════════════════════════════════════════════════════════════════
static const WifiProfile* findCurrentProfile(String connectedSSID) {
  for (size_t i = 0; i < NETWORKS_COUNT; i++) {
    if (connectedSSID == TAILORED_NETWORKS[i].ssid) {
      return &TAILORED_NETWORKS[i];
    }
  }
  return nullptr;
}

static void syncTimeWithTimezone() {
  Serial.println("[time] Synchronizing network time via NTP...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", TZ_RULE, 1);
  tzset();
  
  struct tm timeinfo;
  int retry = 0;
  while (!getLocalTime(&timeinfo) && retry < 10) {
    Serial.print(".");
    delay(500);
    retry++;
  }
  Serial.println("\n[time] Core time synchronization completed.");
}

static void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == lastConnectedSSID) return;
  
  Serial.println("[wifi] Link down or network transition detected.");
  WiFi.mode(WIFI_STA);
  
  for (size_t i = 0; i < NETWORKS_COUNT; i++) {
    wifiMulti.addAP(TAILORED_NETWORKS[i].ssid, TAILORED_NETWORKS[i].password);
  }
  
  while (wifiMulti.run() != WL_CONNECTED) {
    ledShow((millis() / 300) % 2 ? CRGB::Yellow : CRGB::Black);
    delay(100);
  }
  
  String currentSSID = WiFi.SSID();
  lastConnectedSSID = currentSSID;
  Serial.printf("[wifi] Logged into %s, Local Device IP: %s\n", currentSSID.c_str(), WiFi.localIP().toString().c_str());
  
  const WifiProfile* activeProfile = findCurrentProfile(currentSSID);
  
  if (activeProfile != nullptr && activeProfile->locationType == LOCATION_LOCAL) {
    Serial.println("[network] Local interface active. Interfacing server over raw LAN.");
    activeServerHost = AgentConfig::LOCAL_IP;
  } 
  else {
    Serial.println("[network] Remote interface active. Building WireGuard secure pipeline...");
    syncTimeWithTimezone();
    
    bool wgConnected = wg.begin(
      IPAddress().fromString(WireGuardConfig::INTERNAL_IP),
      WireGuardConfig::PRIVATE_KEY,
      WireGuardConfig::PUBLIC_ENDPOINT,
      WireGuardConfig::SERVER_PUB_KEY,
      WireGuardConfig::UDP_PORT
    );
    
    if (wgConnected) {
      Serial.println("[network] WireGuard handshake operational.");
      activeServerHost = AgentConfig::TUNNEL_IP;
    } else {
      Serial.println("[network] CRITICAL ERROR: WireGuard configuration pipeline failure.");
    }
  }
}

static bool sendFrame(const char *tag, const uint8_t *payload = nullptr, uint32_t len = 0) {
  if (!client.connected()) return false;
  FrameHeader hdr;
  memcpy(hdr.tag, tag, 4);
  hdr.length = len;                       
  if (client.write((uint8_t *)&hdr, sizeof(hdr)) != sizeof(hdr)) return false;
  if (len && client.write(payload, len) != len) return false;
  return true;
}

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
// Button — debounced press detection
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
    if (down) return true;      
  }
  return false;
}

// ═════════════════════════════════════════════════════════════════════════════
// State transitions
// ═════════════════════════════════════════════════════════════════════════════
static void setState(State s) {
  if (s == state) return;
  Serial.printf("[state] %s -> %s\n", stateName(state), stateName(s));
  state = s;
  ledUpdate();

  switch (s) {
    case State::IDLE:      micStreaming = false; break;
    case State::WAITING:   micStreaming = false; beep(1000, 0.08f, 0.3f); break;
    case State::LISTENING: break;
    case State::THINKING:
      micStreaming = false;
      playSilence(0.15f);
      beep(1000, 0.12f, 0.3f);
      break;
    case State::SPEAKING:  micStreaming = false; break;
    case State::CLOSING:
      micStreaming = false;
      spkDrain();
      closingBeeps();
      break;
  }
}

static void enterListening() {
  beep(700, 0.12f, 0.3f);       
  spkDrain();
  i2sSetMode(I2SMode::MIC);
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
  i2sSetMode(I2SMode::SPEAKER);   
  Serial.println("[satellite] IDLE - press button to start a session");
}

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
// Mic pump
// ═════════════════════════════════════════════════════════════════════════════
static void pumpMic() {
  static int16_t buf[MIC_CHUNK];
  size_t bytesRead = 0;
  if (i2s_read(I2S_NUM_0, buf, sizeof(buf), &bytesRead, pdMS_TO_TICKS(100)) != ESP_OK
      || bytesRead == 0) return;

  size_t n = bytesRead / sizeof(int16_t);
  for (size_t i = 0; i < n; i++) {
    int32_t v = (int32_t)(buf[i] * MIC_GAIN);
    buf[i] = (int16_t)constrain(v, -32768, 32767);
  }
  if (!sendFrame("AUD0", (uint8_t *)buf, n * sizeof(int16_t)))
    Serial.println("[mic] send error");
}

// ═════════════════════════════════════════════════════════════════════════════
// Incoming audio streaming
// ═════════════════════════════════════════════════════════════════════════════
static bool streamAudioPayload(uint32_t length) {
  static uint8_t buf[STREAM_CHUNK];

  if (!discardTTS) {
    i2sSetMode(I2SMode::SPEAKER);
    if (state != State::SPEAKING && sessionActive) setState(State::SPEAKING);
  }

  uint32_t remaining = length;
  while (remaining) {
    uint32_t n = min((uint32_t)STREAM_CHUNK, remaining);
    if (!readExactly(buf, n, 15000)) return false;
    remaining -= n;

    if (!discardTTS) {
      size_t samples = n / sizeof(int16_t);
      applyVolume((int16_t *)buf, samples);
      spkWrite((int16_t *)buf, samples);

      if (buttonPressed() && sessionActive) hangUp();   
    }
    ledUpdate();
  }
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// Frame dispatch
// ═════════════════════════════════════════════════════════════════════════════
static bool skipPayload(uint32_t length) {
  static uint8_t buf[256];
  while (length) {
    uint32_t n = min((uint32_t)sizeof(buf), length);
    if (!readExactly(buf, n)) return false;
    length -= n;
  }
  return true;
}

static bool pumpSocket() {
  if (client.available() < (int)sizeof(FrameHeader)) return client.connected();

  FrameHeader hdr;
  if (!readExactly((uint8_t *)&hdr, sizeof(hdr))) return false;
  lastEventMs = millis();

  if (tagIs(hdr.tag, "TTS0") || tagIs(hdr.tag, "BEEP")) {
    return streamAudioPayload(hdr.length);
  }

  if (tagIs(hdr.tag, "RDY0")) {
    if (hdr.length && !skipPayload(hdr.length)) return false;
    if (hangingUp) {
      Serial.println("[recv] RDY0 ignored (hanging up)");
      return true;
    }
    Serial.println("[recv] RDY0");
    spkDrain();                       
    if (!sessionActive) {
      sessionActive = true;
      hangingUp     = false;
      discardTTS    = false;
    }
    enterListening();
    return true;
  }

  if (tagIs(hdr.tag, "THNK")) {
    if (hdr.length && !skipPayload(hdr.length)) return false;
    if (hangingUp) return true;       
    if (state != State::THINKING) {
      Serial.println("[recv] THNK");
      setState(State::THINKING);
    }
    return true;
  }

  if (tagIs(hdr.tag, "CLOS")) {
    if (hdr.length && !skipPayload(hdr.length)) return false;
    Serial.println("[recv] CLOS - session ended, connection persists");
    discardTTS = false;               
    setState(State::CLOSING);
    endSessionToIdle();
    return true;
  }

  if (tagIs(hdr.tag, "CALL")) {
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

  Serial.printf("[recv] Unknown tag %.4s (%u bytes)\n", hdr.tag, (unsigned)hdr.length);
  return skipPayload(hdr.length);
}

static void handleButton() {
  if (!buttonPressed()) return;
  if (!client.connected()) return;

  if (state == State::IDLE) {
    if (millis() - lastSessionEnd < WAKE_COOLDOWN_MS) return;   
    Serial.println("[button] wake -> starting session");
    startSession(nullptr, 0);
  } else if (sessionActive) {
    hangUp();
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// Connection lifecycle
// ═════════════════════════════════════════════════════════════════════════════
static bool connectToServer() {
  Serial.printf("[satellite] Reaching voice engine via %s:%d...\n", activeServerHost.c_str(), activeServerPort);
  if (!client.connect(activeServerHost.c_str(), activeServerPort, 5000)) return false;
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
      pumpMic();                      

    if (!pumpSocket()) break;         

    if (hangingUp && millis() - hangupSentMs > HANGUP_FALLBACK_MS) {
      Serial.println("[session] No CLOS after BYE0 - closing locally.");
      discardTTS = false;
      setState(State::CLOSING);
      endSessionToIdle();
    }

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

// ═════════════════════════════════════════════════════════════════════════════
// Arduino entry points
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT);         

  FastLED.addLeds<SK6812, PIN_LED, GRB>(led, 1);
  FastLED.setBrightness(LED_BRIGHTNESS);
  ledShow(CRGB::Black);

  Serial.printf("\n[satellite] Starting up (endpoint_id=\"%s\")...\n", ENDPOINT_ID);

  i2sSetMode(I2SMode::SPEAKER);
  playSilence(0.1f);                  
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
  reconnectDelay = 2000.0f;           

  runConnection();

  Serial.printf("[satellite] Disconnected. Reconnecting in %.0fs...\n",
                reconnectDelay / 1000.0f);
  uint32_t until = millis() + (uint32_t)reconnectDelay;
  while (millis() < until) { ledUpdate(); delay(50); }
  reconnectDelay = min(reconnectDelay * 2, 60000.0f);
}