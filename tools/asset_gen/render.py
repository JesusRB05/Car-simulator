# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
"""Render previews of a .glb with three.js in headless Chromium.

usage: python3 render.py model.glb out_prefix "name:key=val&key=val" ...
"""
import http.server
import os
import shutil
import socketserver
import sys
import threading

from playwright.sync_api import sync_playwright

VIEW = os.path.join(os.path.dirname(os.path.abspath(__file__)), "view")


def serve():
    handler = lambda *a, **k: http.server.SimpleHTTPRequestHandler(*a, directory=VIEW, **k)
    httpd = socketserver.TCPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def main():
    model, prefix, shots = sys.argv[1], sys.argv[2], sys.argv[3:]
    shutil.copy(model, os.path.join(VIEW, "model.glb"))
    httpd = serve()
    port = httpd.server_address[1]
    with sync_playwright() as p:
        b = p.chromium.launch(args=["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"])
        for shot in shots:
            name, query = shot.split(":", 1)
            w, h = 1600, 900
            for kv in query.split("&"):
                if kv.startswith("w="): w = int(kv[2:])
                if kv.startswith("h="): h = int(kv[2:])
            page = b.new_page(viewport={"width": w, "height": h})
            page.on("console", lambda m: print("console:", m.text) if m.type in ("error", "warning") else None)
            page.goto(f"http://127.0.0.1:{port}/viewer.html?model=model.glb&{query}")
            page.wait_for_function("window.__ready === true", timeout=600000)
            err = page.evaluate("window.__error || null")
            if err:
                print("ERROR", err)
            page.wait_for_timeout(500)
            page.screenshot(path=f"{prefix}_{name}.png", timeout=600000)
            print("wrote", f"{prefix}_{name}.png")
            page.close()
        b.close()
    httpd.shutdown()


if __name__ == "__main__":
    main()
