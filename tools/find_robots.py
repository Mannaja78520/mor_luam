"""Find mor_luam robots (and other modules) on the Wi-Fi this PC is on.

    python tools/find_robots.py                    search every local /24 network
    python tools/find_robots.py --subnet 192.168.137.0/24
    docker\\mor_luam.bat find                       the same, from the helper

How: asks every address for http://<ip>/api/whoami (each robot answers
{"type":"morluam", "name", "host", "ip", "fw"}), and also tries the mDNS names
mor-luam.local / the hostname given with --name. Robots announce themselves
on mDNS as _module._tcp too (like the mice project modules); if the Python
package "zeroconf" is installed, that is browsed as well.

Standard library only (zeroconf optional). Nothing is sent but GET requests.
"""
import argparse
import concurrent.futures as cf
import ipaddress
import json
import socket
import subprocess
import sys
import time
import urllib.request


PORT = 80


def whoami(ip, timeout=0.8):
    try:
        with urllib.request.urlopen(f"http://{ip}:{PORT}/api/whoami", timeout=timeout) as r:
            d = json.loads(r.read().decode("utf-8"))
            return d if isinstance(d, dict) and d.get("type") else None
    except Exception:
        return None


def local_networks():
    """IPv4 /24 networks of this PC's interfaces (Windows: from ipconfig)."""
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    if sys.platform == "win32":
        try:
            out = subprocess.run(["ipconfig"], capture_output=True, text=True, errors="ignore").stdout
            for line in out.splitlines():
                if "IPv4" in line and ":" in line:
                    ips.add(line.split(":")[-1].strip())
        except OSError:
            pass
    nets = set()
    for ip in ips:
        try:
            a = ipaddress.ip_address(ip)
        except ValueError:
            continue
        if a.is_loopback or a.is_link_local:
            continue
        nets.add(ipaddress.ip_network(f"{ip}/24", strict=False))
    return sorted(nets, key=str)


def resolve(name):
    try:
        return socket.gethostbyname(name)
    except OSError:
        return None


def browse_zeroconf(seconds=3.0):
    try:
        from zeroconf import ServiceBrowser, Zeroconf
    except ImportError:
        return []
    found = []

    class Listener:
        def add_service(self, zc, type_, name):
            info = zc.get_service_info(type_, name, timeout=1500)
            if info and info.parsed_addresses():
                found.append(info.parsed_addresses()[0])

        def update_service(self, *a):
            pass

        def remove_service(self, *a):
            pass

    zc = Zeroconf()
    ServiceBrowser(zc, "_module._tcp.local.", Listener())
    time.sleep(seconds)
    zc.close()
    return found


def main():
    ap = argparse.ArgumentParser(description="Find mor_luam robots on the network.")
    ap.add_argument("--subnet", action="append", help="network to sweep, e.g. 192.168.137.0/24 (repeatable)")
    ap.add_argument("--name", default="mor-luam", help="mDNS hostname to try (default mor-luam)")
    ap.add_argument("--all", action="store_true", help="also list devices that are not mor_luam")
    ap.add_argument("--port", type=int, default=80, help="web port (80 on the robot; 8000 for web/mock_robot.py)")
    a = ap.parse_args()
    global PORT
    PORT = a.port

    nets = [ipaddress.ip_network(s, strict=False) for s in a.subnet] if a.subnet else local_networks()
    candidates = set()
    for n in nets:
        candidates.update(str(h) for h in n.hosts())
    ip = resolve(a.name + ".local")
    if ip:
        candidates.add(ip)
    candidates.update(browse_zeroconf())

    print(f"Searching {len(candidates)} addresses on: {', '.join(map(str, nets)) or '(no network found)'}")
    found = {}
    with cf.ThreadPoolExecutor(max_workers=96) as pool:
        for ip, d in zip(candidates, pool.map(whoami, candidates)):
            if d and (a.all or d.get("type") == "morluam"):
                found[ip] = d

    if not found:
        print("\nNo robot answered. Check:")
        print("  1. the robot is powered and on the same Wi-Fi as this PC (default: the PC hotspot 'manny')")
        print("  2. or connect this PC to the robot's own hotspot mor-luam-XXXX and open http://192.168.4.1")
        print("  3. Windows Firewall / VPN is not blocking local traffic")
        return 1
    print()
    print(f"{'name':<18} {'type':<9} {'IP':<16} {'mDNS name':<22} firmware")
    for ip, d in sorted(found.items(), key=lambda kv: ipaddress.ip_address(kv[0])):
        print(f"{d.get('name', '?'):<18} {d.get('type', '?'):<9} {ip:<16} {d.get('host', ''):<22} {d.get('fw', '')}")
    print("\nOpen the web app:  http://<IP>/   (or http://mor-luam.local/)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
