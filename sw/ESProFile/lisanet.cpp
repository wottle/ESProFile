// LisaNet MODE-line helpers -- NOT part of upstream ESProFile; written with an AI assistant. See lisanet.h.
#include "lisanet.h"

bool lisanetModeStableAt(bool level) {
    static uint32_t since = 0;
    static bool primed = false;
    uint32_t now = millis();
    if (!primed) { since = now; primed = true; }
    if (digitalRead(LISANET_MODE_PIN) != level) { since = now; return false; }
    return now - since >= LISANET_MODE_STABLE_MS;
}

bool lisanetModeRequestedAtBoot() {
    pinMode(LISANET_MODE_PIN, INPUT);
    pinMode(LISANET_LINK_RX_PIN, INPUT);
    // Watch for up to twice the stability window: a request needs MODE high for a full window, with the link input
    // seen high at least once (older bitstreams hold MODE high but the link input low -- see lisanet.h)
    uint32_t start = millis(), since = millis();
    bool linkSeenHigh = false;
    while (millis() - start < 2 * LISANET_MODE_STABLE_MS) {
        if (digitalRead(LISANET_MODE_PIN) == LOW) return false;
        if (digitalRead(LISANET_LINK_RX_PIN) == HIGH) linkSeenHigh = true;
        if (millis() - since >= LISANET_MODE_STABLE_MS) return linkSeenHigh;
        delay(5);
    }
    return linkSeenHigh;
}
