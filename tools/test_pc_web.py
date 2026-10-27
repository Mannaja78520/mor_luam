"""Companion contract: real HTTP with a fake backend, never a physical robot."""
import json
import http.client
import concurrent.futures
import threading
import unittest
import urllib.error
import urllib.request
from unittest.mock import patch
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pc_web import Handler, RobotLink


class Backend(BaseHTTPRequestHandler):
    calls = []
    empty_status_remaining = 0
    def do_GET(self):
        self.calls.append(('GET', self.path))
        data = json.dumps({'robot': {'mode': 'halt'}, 'nav': {}, 'net': {}, 'ros': {}, 'sys': {}}).encode()
        if self.path == '/api/status' and Backend.empty_status_remaining:
            Backend.empty_status_remaining -= 1
            data = b''
        self.send_response(200); self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)
    def do_POST(self):
        self.calls.append(('POST', self.path))
        self.send_response(200); self.end_headers(); self.wfile.write(b'{"ok":true}')
    def log_message(self, *_): pass


class CompanionContract(unittest.TestCase):
    def setUp(self):
        Backend.calls = []
        Backend.empty_status_remaining = 0
        self.backend = ThreadingHTTPServer(('127.0.0.1', 0), Backend)
        self.pc = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.pc.link = RobotLink(discover=False)
        for server in (self.backend, self.pc): threading.Thread(target=server.serve_forever, daemon=True).start()
        self.url = f'http://127.0.0.1:{self.pc.server_port}'
    def tearDown(self):
        for server in (self.pc, self.backend): server.shutdown(); server.server_close()
    def get(self, path):
        with urllib.request.urlopen(self.url + path, timeout=3) as r: return r.read()
    def test_offline_page_and_simulator_need_no_robot(self):
        self.assertIn(b'data-host-mode="pc"', self.get('/'))
        self.assertIn(b'SimulationWaypoints', self.get('/app.js'))
        with self.assertRaises(urllib.error.HTTPError) as e: self.get('/api/status')
        self.assertEqual(e.exception.code, 503)
        self.assertEqual(Backend.calls, [])
    def test_connection_recovery_never_replays_an_offline_move(self):
        request = urllib.request.Request(self.url + '/api/nav/start', data=b'{}', headers={'Content-Type': 'application/json'})
        with self.assertRaises(urllib.error.HTTPError): urllib.request.urlopen(request)
        self.pc.link.target = f'127.0.0.1:{self.backend.server_port}'
        data = json.loads(self.get('/api/status'))
        self.assertEqual(data['robot']['mode'], 'halt')
        self.assertEqual(data['pc']['mode'], 'live')
        self.assertEqual(Backend.calls, [('GET', '/api/status')])
        self.pc.link.target = None
        with self.assertRaises(urllib.error.HTTPError): self.get('/api/status')
        self.assertEqual(len(Backend.calls), 1)
    def test_third_party_form_cannot_send_robot_commands(self):
        self.pc.link.target = f'127.0.0.1:{self.backend.server_port}'
        request = urllib.request.Request(self.url + '/api/nav/start', data=b'{}', headers={'Origin':'https://example.org','Content-Type':'application/json'})
        with self.assertRaises(urllib.error.HTTPError) as e: urllib.request.urlopen(request)
        self.assertEqual(e.exception.code, 403)
        self.assertEqual(Backend.calls, [])

    def test_transient_read_retries_but_post_is_never_repeated(self):
        self.pc.link.target = f'127.0.0.1:{self.backend.server_port}'
        real_open = urllib.request.urlopen
        def flaky(request, **kwargs):
            nonlocal calls
            calls += 1
            if calls == 1: raise failure('lost reply')
            return real_open(request, **kwargs)
        calls = 0
        failure = TimeoutError
        with patch('pc_web.urllib.request.urlopen', side_effect=flaky):
            status, body = RobotLink.request(self.pc.link.target, '/api/status', None, {})
        self.assertEqual((status, calls), (200, 2))
        self.assertEqual(json.loads(body)['robot']['mode'], 'halt')
        calls = 0
        failure = http.client.BadStatusLine
        with patch('pc_web.urllib.request.urlopen', side_effect=flaky):
            status, body = RobotLink.request(self.pc.link.target, '/api/status', None, {})
        self.assertEqual((status, calls), (200, 2))
        calls = 0
        failure = TimeoutError
        with patch('pc_web.urllib.request.urlopen', side_effect=flaky):
            with self.assertRaises(TimeoutError):
                RobotLink.request(self.pc.link.target, '/api/nav/start', b'{}', {})
        self.assertEqual(calls, 1)
        self.assertEqual(Backend.calls, [('GET', '/api/status'), ('GET', '/api/status')])

    def test_open_tabs_share_one_inflight_status_read(self):
        self.pc.link.target = f'127.0.0.1:{self.backend.server_port}'
        entered, release = threading.Event(), threading.Event()
        def delayed(*args):
            entered.set()
            self.assertTrue(release.wait(2))
            return 200, b'{"robot":{"mode":"halt"}}'
        with patch.object(self.pc.link, 'request', side_effect=delayed) as request:
            with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
                first = pool.submit(self.pc.link.forward, '/api/status')
                self.assertTrue(entered.wait(1))
                rest = [pool.submit(self.pc.link.forward, '/api/status') for _ in range(3)]
                # All callers have entered forward before the real reply completes.
                for future in rest: self.assertFalse(future.done())
                release.set()
                for future in [first, *rest]: self.assertEqual(future.result()[0], 200)
            self.assertEqual(request.call_count, 1)

    def test_empty_success_reply_is_retried_not_forwarded_as_live_status(self):
        self.pc.link.target = f'127.0.0.1:{self.backend.server_port}'
        Backend.empty_status_remaining = 1
        data = json.loads(self.get('/api/status'))
        self.assertEqual(data['robot']['mode'], 'halt')
        self.assertEqual(Backend.calls, [('GET', '/api/status'), ('GET', '/api/status')])

    def test_local_networks_returns_valid_subnets_without_loopback(self):
        from pc_web import local_networks
        nets = local_networks()
        self.assertIsInstance(nets, list)
        for net in nets:
            self.assertFalse(net.is_loopback)
            self.assertFalse(net.is_link_local)


if __name__ == '__main__': unittest.main()
