"""Serve the mor_luam UI on this PC, including an offline simulator.

Robot APIs are available only while a real mor_luam answers discovery. No fake
telemetry, background heartbeat or automatic movement commands are generated.
Run: python tools/pc_web.py [--port 8000] [--host mor-luam.local] [--no-browser]
"""
import argparse
import http.client
import concurrent.futures
import ipaddress
import itertools
import json
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

WEB = Path(__file__).resolve().parents[1] / 'firmware' / 'web'


def local_networks():
    """IPv4 /24 networks of this PC's interfaces (Windows: from ipconfig; cross-platform: socket)."""
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

    def sort_key(n):
        s = str(n)
        return (0 if s.startswith("192.168.") else 1 if s.startswith("172.") else 2, s)

    return sorted(nets, key=sort_key)


class RobotLink:
    def __init__(self, hostname='mor-luam.local', subnet=None, discover=True):
        self.hostname, self.subnet = hostname, subnet
        self.target = None
        self.lock = threading.Lock()
        self.jobs = queue.PriorityQueue()
        self.sequence = itertools.count()
        self.pending_status = None
        self.last_response = 0
        threading.Thread(target=self.worker, daemon=True).start()
        if discover:
            threading.Thread(target=self.discovery, daemon=True).start()

    @staticmethod
    def identify(host):
        try:
            with urllib.request.urlopen('http://' + host + '/api/whoami', timeout=.7) as r:
                return json.load(r).get('type') == 'morluam'
        except (OSError, ValueError, http.client.HTTPException):
            return False

    @staticmethod
    def request(target, path, body, headers):
        # A lost read is safe to repeat. Never repeat a motor/settings POST:
        # its reply may be lost after the robot has already applied it.
        attempts = 2 if body is None else 1
        for attempt in range(attempts):
            request = urllib.request.Request('http://' + target + path, data=body, headers=headers,
                                             method='POST' if body is not None else 'GET')
            try:
                with urllib.request.urlopen(request, timeout=1 if body is None else 2) as response:
                    data = response.read()
                    if body is None and path == '/api/status':
                        obj = json.loads(data)
                        if not isinstance(obj, dict) or any(not isinstance(obj.get(key), dict)
                                                           for key in ('robot', 'nav', 'net', 'ros', 'sys')):
                            raise ValueError('Incomplete robot status')
                    return response.status, data
            except urllib.error.HTTPError:
                raise
            except (OSError, ValueError, http.client.HTTPException):
                if attempt + 1 == attempts:
                    raise

    def discovery(self):
        last_scan = 0
        while True:
            if self.target and time.monotonic() - self.last_response < 5:
                time.sleep(2)
                continue  # Real API replies already confirm this robot is here.
            candidates = [self.target]
            try:
                candidates.append(socket.gethostbyname(self.hostname))
            except OSError:
                pass
            found = next((h for h in dict.fromkeys(candidates) if h and self.identify(h)), None)
            if not found and time.monotonic() - last_scan > 12:
                last_scan = time.monotonic()
                subnets = [ipaddress.ip_network(self.subnet, strict=False)] if self.subnet else local_networks()
                hosts = list(dict.fromkeys(str(h) for net in subnets for h in net.hosts()))
                with concurrent.futures.ThreadPoolExecutor(max_workers=64) as pool:
                    found = next((h for h, valid in zip(hosts, pool.map(self.identify, hosts)) if valid), None)
            with self.lock:
                self.target = found
            time.sleep(2)

    def worker(self):
        while True:
            _, _, created, target, path, body, headers, future = self.jobs.get()
            if not future.set_running_or_notify_cancel():
                continue
            if self.target != target or (body is not None and time.monotonic() - created > 3):
                future.set_result((503, b'{"ok":false,"error":"Robot connection changed; retry when ready"}'))
                continue
            try:
                status, data = self.request(target, path, body, headers)
                self.last_response = time.monotonic()
                if path == '/api/status':
                    obj = json.loads(data)
                    obj['pc'] = {'robotHost': target, 'mode': 'live'}
                    data = json.dumps(obj, ensure_ascii=False).encode('utf-8')
                if not future.cancelled():
                    future.set_result((status, data))
            except urllib.error.HTTPError as error:
                if not future.cancelled():
                    future.set_result((error.code, error.read()))
            except (OSError, ValueError, http.client.HTTPException):
                # One missing reply does not prove Wi-Fi disconnected. Keep
                # discovery's verified target if its small identity reply works.
                if not self.identify(target):
                    with self.lock:
                        if self.target == target:
                            self.target = None
                if not future.cancelled():
                    future.set_result((503, b'{"ok":false,"error":"Robot offline; simulator is still available"}'))

    def forward(self, path, body=None, headers=None):
        shared_status = path == '/api/status' and body is None
        with self.lock:
            target = self.target
            if not target:
                return 503, b'{"ok":false,"error":"Robot offline; simulator is still available"}'
            pending = self.pending_status
            if shared_status and pending and pending[0] == target and not pending[1].done():
                future = pending[1]  # Open tabs share one in-flight real status read.
            else:
                future = concurrent.futures.Future()
                if shared_status:
                    self.pending_status = (target, future)
                urgent = path.startswith(('/api/estop', '/api/nav/stop'))
                realtime = body is not None or shared_status
                self.jobs.put((-1 if urgent else 0 if realtime else 5, next(self.sequence), time.monotonic(),
                               target, path, body, headers or {}, future))
        try:
            return future.result(timeout=8)
        except concurrent.futures.TimeoutError:
            if not shared_status:
                future.cancel()
            return 504, b'{"ok":false,"error":"Robot did not answer; command not retried"}'


class Handler(BaseHTTPRequestHandler):
    def send_bytes(self, data, content_type='application/json; charset=utf-8', status=200):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(data)
        except ConnectionError:
            pass

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/pc/status':
            return self.send_bytes(json.dumps({'type': 'morluam-pc', 'robotHost': self.server.link.target}).encode())
        if path.startswith('/api/'):
            status, data = self.server.link.forward(self.path)
            return self.send_bytes(data, status=status)
        if path == '/':
            data = (WEB / 'index.html').read_text(encoding='utf-8').replace('<html lang="th">', '<html lang="th" data-host-mode="pc">')
            return self.send_bytes(data.encode('utf-8'), 'text/html; charset=utf-8')
        if path == '/app.css':
            return self.send_bytes((WEB / 'app.css').read_bytes(), 'text/css; charset=utf-8')
        if path == '/app.js':
            data = b'\n'.join(p.read_bytes() for p in sorted((WEB / 'js').glob('*.js')))
            return self.send_bytes(data, 'application/javascript; charset=utf-8')
        self.send_error(404)

    def do_POST(self):
        if not urlsplit(self.path).path.startswith('/api/'):
            return self.send_error(404)
        # Browser requests must come from this local UI, not a third-party form.
        origin = self.headers.get('Origin')
        if origin and urlsplit(origin).netloc != self.headers.get('Host'):
            return self.send_error(403)
        length = int(self.headers.get('Content-Length', '0'))
        if length > 2 * 1024 * 1024:
            return self.send_error(413)
        body = self.rfile.read(length)
        headers = {'Content-Type': self.headers.get('Content-Type', 'application/json')}
        # OTA must be performed on the robot page/helper while halted; never
        # hold the normal control/heartbeat queue behind a long firmware upload.
        if urlsplit(self.path).path == '/api/ota':
            return self.send_bytes(b'{"ok":false,"error":"Use fw-ota or the direct robot page for firmware updates"}', status=409)
        status, data = self.server.link.forward(self.path, body, headers)
        self.send_bytes(data, status=status)

    def log_message(self, *_):
        pass  # API responses/settings/credentials never enter console logs.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='mor-luam.local')
    parser.add_argument('--subnet', default=None,
                        help='Subnet to scan (e.g. 192.168.137.0/24). If omitted, automatically scans all local network interfaces.')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    url = f'http://localhost:{args.port}/'
    try:
        with urllib.request.urlopen(f'http://127.0.0.1:{args.port}/pc/status', timeout=.3) as response:
            existing = json.load(response)
        if existing.get('type') == 'morluam-pc':
            print('mor_luam PC web is already running: ' + url, flush=True)
            if not args.no_browser:
                webbrowser.open(url)
            return
    except (OSError, ValueError):
        pass
    link = RobotLink(args.host, args.subnet)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.link = link
    print('mor_luam web on this PC: ' + url, flush=True)
    print('Simulation works offline. A real robot connects automatically; no movement starts automatically.', flush=True)
    if not args.no_browser:
        threading.Timer(.2, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
