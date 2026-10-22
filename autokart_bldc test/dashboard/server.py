"""Live, read-only browser dashboard for the Pico experiment."""
from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ASSETS = Path(__file__).resolve().parent
HTML_PAGE = (ASSETS / "index.html").read_bytes()
STYLE_CSS = (ASSETS / "style.css").read_bytes()
APP_JS = (ASSETS / "app.js").read_bytes()

class AppState:
    """Shared, lock-protected snapshot the HTTP handler reads and the main loop writes."""

    def __init__(self):
        self._lock = threading.Lock()
        self._status: dict = {}
        self._jpeg: bytes | None = None
        self.pending_actions: list[str] = []

    def update(self, status: dict, jpeg: bytes | None = None):
        with self._lock:
            self._status = status
            if jpeg is not None:
                self._jpeg = jpeg

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._status)

    def latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def push_action(self, action: str):
        with self._lock:
            self.pending_actions.append(action)

    def drain_actions(self) -> list[str]:
        with self._lock:
            actions, self.pending_actions = self.pending_actions, []
            return actions


def make_handler(state: AppState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, code: int, body: bytes, content_type: str):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/" or self.path.startswith("/?"):
                self._send(200, HTML_PAGE, "text/html; charset=utf-8")
            elif self.path == "/style.css":
                self._send(200, STYLE_CSS, "text/css; charset=utf-8")
            elif self.path == "/app.js":
                self._send(200, APP_JS, "application/javascript; charset=utf-8")
            elif self.path == "/api/status":
                self._send(200, json.dumps(state.snapshot()).encode(), "application/json")
            elif self.path.startswith("/snapshot.jpg"):
                jpeg = state.latest_jpeg()
                if jpeg is None:
                    self._send(503, b"{}", "application/json")
                else:
                    self._send(200, jpeg, "image/jpeg")
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            if self.path == "/api/control":
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length) if length else b"{}"
                try:
                    action = json.loads(body).get("action")
                except json.JSONDecodeError:
                    action = None
                if action:
                    state.push_action(action)
                self._send(200, b"{}", "application/json")
            else:
                self._send(404, b"not found", "text/plain")

    return Handler


def run_server(state: AppState, host: str, port: int) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), make_handler(state))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server
