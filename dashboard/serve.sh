#!/bin/sh
# Serve the dashboard on http://localhost:8000 (Web Serial needs localhost or https)
cd "$(dirname "$0")" && echo "Open http://localhost:8000 in Chrome/Chromium" && python3 -m http.server 8000 --bind 127.0.0.1
