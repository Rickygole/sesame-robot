// secrets_example.h -- copy to secrets.h and edit.
//
//     cp firmware/sesame/secrets_example.h firmware/sesame/secrets.h
//
// secrets.h is gitignored. This repository is PUBLIC, so a WiFi password
// committed here is a WiFi password published to the internet -- and git
// history keeps it even after a later deletion.
#pragma once

namespace sesame {

// The robot always creates its own access point, so it stays reachable
// even if joining your network fails. Connect to this and browse to
// http://192.168.4.1
//
// WPA2 requires at least 8 characters. Change it: this default is in a
// public repository, so anyone within radio range knows it.
constexpr char kApSsid[] = "Sesame";
constexpr char kApPassword[] = "sesame123";

// OPTIONAL: join your existing WiFi as well, so the companion app can
// reach the robot without leaving your network.
//
// Leave kStationSsid empty ("") to skip this entirely.
// ESP32 has no 5GHz radio -- this MUST be a 2.4GHz network.
constexpr char kStationSsid[] = "";
constexpr char kStationPassword[] = "";

// Reachable at http://sesame.local once mDNS is up. On Windows this
// needs Bonjour; some routers block mDNS, in which case use the IP that
// is printed to serial at boot.
constexpr char kHostname[] = "sesame";

}  // namespace sesame
