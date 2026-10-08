#include "net/WifiStore.h"
#include <string.h>

uint8_t WifiStore::count() {
    lock();
    const uint8_t n = count_;
    unlock();
    return n;
}

bool WifiStore::entry(uint8_t i, Entry& out) {
    lock();
    const bool ok = i < count_;
    if (ok) out = entries_[i];
    unlock();
    return ok;
}

int16_t WifiStore::indexOf(const char* ssid) const {
    for (uint8_t i = 0; i < count_; ++i)
        if (strncmp(entries_[i].ssid, ssid, SSID_MAX) == 0) return i;
    return -1;
}

void WifiStore::copy(Entry& dst, const char* ssid, const char* pass) {
    strncpy(dst.ssid, ssid ? ssid : "", SSID_MAX - 1);
    dst.ssid[SSID_MAX - 1] = '\0';
    strncpy(dst.pass, pass ? pass : "", PASS_MAX - 1);
    dst.pass[PASS_MAX - 1] = '\0';
}

bool WifiStore::set(const char* ssid, const char* pass) {
    if (!ssid || !ssid[0] || strlen(ssid) >= SSID_MAX) return false;
    if (pass && strlen(pass) >= PASS_MAX) return false;
    lock();
    bool ok = false;
    const int16_t at = indexOf(ssid);
    if (at >= 0) {
        copy(entries_[at], ssid, pass);          // replace, never duplicate
        ok = save();
    } else if (count_ < MAX) {
        copy(entries_[count_++], ssid, pass);
        ok = save();
    }
    unlock();
    return ok;
}

bool WifiStore::remove(const char* ssid) {
    lock();
    const int16_t at = indexOf(ssid);
    bool ok = false;
    if (at >= 0) {
        for (uint8_t i = at; i + 1 < count_; ++i) entries_[i] = entries_[i + 1];
        --count_;
        ok = save();
    }
    unlock();
    return ok;
}

bool WifiStore::moveTo(const char* ssid, uint8_t to) {
    lock();
    const int16_t from = indexOf(ssid);
    bool ok = false;
    if (from >= 0 && to < count_) {
        Entry held = entries_[from];
        if (to > from) for (uint8_t i = from; i < to; ++i) entries_[i] = entries_[i + 1];
        else for (uint8_t i = from; i > to; --i) entries_[i] = entries_[i - 1];
        entries_[to] = held;
        ok = save();
    }
    unlock();
    return ok;
}

void WifiStore::toJson(JsonArray out) {
    lock();
    for (uint8_t i = 0; i < count_; ++i) {
        JsonObject o = out.add<JsonObject>();
        o["ssid"] = entries_[i].ssid;
        o["pass"] = entries_[i].pass;
    }
    unlock();
}

// One blob, not a key per field: it cannot be half-written into a broken list.
bool WifiStore::save() {
    prefs_.putUChar("n", count_);
    if (count_ == 0) return true;
    const size_t want = sizeof(Entry) * count_;
    return prefs_.putBytes("list", entries_, want) == want;
}

bool WifiStore::load() {
    if (!prefs_.isKey("n")) return false;
    const uint8_t n = prefs_.getUChar("n", 0);
    if (n > MAX) return false;
    if (n == 0) { count_ = 0; return true; }      // emptied on purpose: do not re-seed
    const size_t want = sizeof(Entry) * n;
    if (prefs_.getBytesLength("list") != want) return false;
    prefs_.getBytes("list", entries_, want);
    for (uint8_t i = 0; i < n; ++i) {
        entries_[i].ssid[SSID_MAX - 1] = '\0';
        entries_[i].pass[PASS_MAX - 1] = '\0';
    }
    count_ = n;
    return true;
}
