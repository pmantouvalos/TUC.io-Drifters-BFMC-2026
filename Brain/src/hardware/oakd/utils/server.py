from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from time import sleep

import cv2

class VideoStreamHandler(BaseHTTPRequestHandler):

    def do_GET(self):

        #print("New connection")
        
        self.send_response(200)
        self.send_header(
            "Content-type", "multipart/x-mixed-replace; boundary=--jpgboundary"
        )
        self.end_headers()
        
        try:
            while True:
                sleep(self.server.sleep_s)
                if hasattr(self.server, "frametosend"):
                    ok, encoded = cv2.imencode(".jpg", self.server.frametosend, [cv2.IMWRITE_JPEG_QUALITY, self.server.quality])#self.server.quality
                    self.wfile.write("--jpgboundary".encode())
                    self.send_header("Content-type", "image/jpeg")
                    self.send_header("Content-length", str(len(encoded)))
                    self.end_headers()
                    self.wfile.write(encoded)
                    self.end_headers()
        except:
            #print("Client Disconnected")
            pass

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):    
    """Handle requests in a separate thread."""
    
    def __init__(self, *args, **kwargs):

        # Setup stream specific parameters
        self.sleep_s = 0.08
        self.quality = 80
        
        super().__init__(*args, **kwargs)