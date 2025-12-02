#!/usr/bin/env python3
"""
Simple HTTP server with COOP/COEP headers for SharedArrayBuffer support.
Required for Emscripten pthreads to work.
"""

# Note: The pthreads headers are unused as we only are running one thread currently

from http.server import HTTPServer, SimpleHTTPRequestHandler
import mimetypes
import sys

class CORSRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        # Required headers for SharedArrayBuffer (needed for pthreads)
        self.send_header('Cross-Origin-Opener-Policy', 'same-origin')
        self.send_header('Cross-Origin-Embedder-Policy', 'require-corp')
        # Allow loading local resources
        self.send_header('Cross-Origin-Resource-Policy', 'cross-origin')
        # Ensure wasm files are served with correct MIME type for streaming compile
        mimetypes.add_type('application/wasm', '.wasm')
        super().end_headers()

    def log_message(self, format, *args):
        # Colorful logging
        print(f"\033[92m[{self.log_date_time_string()}]\033[0m {format % args}")

if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    server = HTTPServer(('', port), CORSRequestHandler)
    print(f"\033[96m╔═══════════════════════════════════════╗\033[0m")
    print(f"\033[96m║        Tactility Simulator.js         ║\033[0m")
    print(f"\033[96m║ ------------------------------------- ║\033[0m")
    print(f"\033[96m║ Go to:                                ║\033[0m")
    print(f"\033[96m║ http://localhost:{port:<20} ║\033[0m")
    print(f"\033[96m╚═══════════════════════════════════════╝\033[0m")
    print(f"\033[93mPress Ctrl+C to stop the server\033[0m\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print(f"\n\033[91mServer stopped.\033[0m")
        sys.exit(0)