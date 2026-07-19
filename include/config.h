#pragma once
#include <Arduino.h>

// ── Identity (Retained from your original satellite footprint) ──────────────
#define ENDPOINT_ID     "Jesse-mobile1"
#define FRIENDLY_NAME   "Jesse Mobile Wearable Interface"

// ── Audio Tuning (Retained from your original hardware profile) ──────────────
#define MIC_GAIN        4.0f    // 1.0 = raw. 3–6 is a sensible range.
#define SPK_VOLUME      1.0f    // 0.0–1.0

// ── Interface Mechanics (Retained from your original settings) ──────────────
#define DEBOUNCE_MS        30
#define HANGUP_FALLBACK_MS 5000
#define WAKE_COOLDOWN_MS   800
#define SESSION_TIMEOUT_MS 60000   
#define LED_BRIGHTNESS     60      

// ── Network Location Routing Types ───────────────────────────────────────────
enum NetworkLocation {
    LOCATION_LOCAL,  // Direct connection to local server (low latency)
    LOCATION_REMOTE  // Needs to spin up WireGuard tunnel to reach home
};

struct WifiProfile {
    const char* ssid;
    const char* password;
    NetworkLocation locationType;
};

// ── Multi-Wi-Fi Profile Registry ─────────────────────────────────────────────
// The system scans and targets the strongest profile available automatically.
const WifiProfile TAILORED_NETWORKS[] = {
    {"home-wifi", "password", LOCATION_LOCAL},
    {"work-wifi", "password", LOCATION_REMOTE},
    {"mobile-wifi", "password", LOCATION_REMOTE}
};
const size_t NETWORKS_COUNT = sizeof(TAILORED_NETWORKS) / sizeof(TAILORED_NETWORKS[0]);

// ── WireGuard VPN Credentials ────────────────────────────────────────────────
// Extracted from your home DietPi deployment client profiles
namespace WireGuardConfig {
    const char PRIVATE_KEY[]     = "private_key";
    const char INTERNAL_IP[]     = "10.22.245.4";                        // Unique client VPN IP
    const char SERVER_PUB_KEY[]  = "public_key";     // Server verification key
    const char PUBLIC_ENDPOINT[] = "home_public_ip";            // Home public IP or DDNS
    const int  UDP_PORT          = 51820;                            // Forwarded UDP port
    const char LAN_SUBNET[]      = "192.168.8.0/24";          // set this so that the wireguard client knows to route remote LAN traffic through
}

// ── Dynamic Server Core Connections ──────────────────────────────────────────
namespace AgentConfig {
    const char LOCAL_IP[]   = "jetson.lan";  // Target address when directly at home
    const char TUNNEL_IP[]  = "192.168.8.200";    // Target address (DietPi server IP) inside VPN tunnel
    const int  SERVER_PORT  = 10400;
}

// ── POSIX Timezone Rule ──────────────────────────────────────────────────────
// Configured for Melbourne/Sydney (AEST/AEDT) to automate Daylight Savings shifts.
const char* TZ_RULE = "AEST-10AEDT,M10.1.0,M4.1.0/3";