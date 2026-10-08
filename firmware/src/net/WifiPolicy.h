#pragma once
// Which saved Wi-Fi to join, and when to move to a better one.
// The order of the saved list (web page, Wi-Fi tab, the up arrow) IS the priority.
//
//   after a drop    1. the network the robot was on, if it is on the air
//                   2. then the saved list top to bottom (priority 1, 2, 3 ...)
//                   each one that does not answer -> the next one
//   while joined    on priority 2, 3 ... and a network higher in the list is on
//                   the air (strong enough) -> move up to it. NetworkManager only
//                   does this while the robot stands still: the move drops the
//                   link for a few seconds.
//
// Plain C++ (no Arduino) so test_host/tests.cpp can check it on a PC.
#include <vector>

namespace wifipolicy {

static const int NOT_SEEN = -1000;   // rssi of a saved network that is not on the air

// rssi[k]: signal of saved network k (dBm) or NOT_SEEN; last: index joined before, -1 if none.
// Returns saved indices in the order to try. Networks not on the air are left out.
inline std::vector<int> joinOrder(const std::vector<int>& rssi, int last) {
    std::vector<int> order;
    const int n = (int)rssi.size();
    if (last >= 0 && last < n && rssi[last] != NOT_SEEN) order.push_back(last);
    for (int k = 0; k < n; ++k)
        if (k != last && rssi[k] != NOT_SEEN) order.push_back(k);
    return order;
}

// Joined at saved index cur (-1: a network no longer in the list, i.e. the lowest).
// Returns the highest-priority network above it worth moving to, or -1.
inline int moveUpTo(const std::vector<int>& rssi, int cur, int minRssi) {
    const int n = (int)rssi.size();
    const int above = (cur < 0 || cur > n) ? n : cur;
    for (int k = 0; k < above; ++k)
        if (rssi[k] != NOT_SEEN && rssi[k] >= minRssi) return k;
    return -1;
}

}  // namespace wifipolicy
