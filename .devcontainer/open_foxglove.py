from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import quote

class RedirectHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        host = self.headers["Host"]

        # Convert the forwarded 8080 hostname to the simulator's 8765 hostname.
        websocket_host = host.replace("-8080.", "-8765.")
        websocket_url = f"wss://{websocket_host}"

        foxglove_url = (
            "https://app.foxglove.dev/"
            "?ds=foxglove-websocket"
            f"&ds.url={quote(websocket_url, safe='')}"
        )

        self.send_response(302)
        self.send_header("Location", foxglove_url)
        self.end_headers()

    def log_message(self, format, *args):
        pass

HTTPServer(("0.0.0.0", 8080), RedirectHandler).serve_forever()
