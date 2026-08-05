#!/usr/bin/env python3
"""Serve the ViDAR browser simulator on http://localhost:8765

Windows:  py -3 scripts/serve_sim.py
mac/Linux: python3 scripts/serve_sim.py
"""

from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent.parent / "sim"
PORT = 8765


class SimHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(SIM_DIR), **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def main():
    if not SIM_DIR.is_dir():
        raise SystemExit(f"Sim folder not found: {SIM_DIR}")

    server = ThreadingHTTPServer(("127.0.0.1", PORT), SimHandler)
    print(f"ViDAR sim → http://127.0.0.1:{PORT}")
    print("Press Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped")
        server.server_close()


if __name__ == "__main__":
    main()
