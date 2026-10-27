"""Companion contract: real HTTP with a fake backend, never a physical robot."""
import json
import threading
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pc_web import Handler, RobotLink


class Backend(BaseHTTPRequestHandler):
    calls = []
    def do_GET(self):
        self.calls.append(('GET', self.path))
        data = json.dumps({'robot': {'mode': 'halt'}}).encode()
        self.send_response(200); self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)
    def do_POST(self):
        self.calls.append(('POST', self.path))
        self.send_response(200); self.end_headers(); self.wfile.write(b'{"ok":true}')
    def log_message(self, *_): pass


class CompanionContract(unittest.TestCase):
    def setUp(self):
        Backend.calls = []
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


if __name__ == '__main__': unittest.main()
