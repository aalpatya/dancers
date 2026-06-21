#!/usr/bin/env bash
# Build the WebAssembly (browser) version of dancers.
#
# Requires the Emscripten SDK: https://emscripten.org/docs/getting_started/downloads.html
# Point EMSDK at your emsdk checkout, or it defaults to ~/emsdk.
#
# Usage:
#   ./build_web.sh          # configure + build -> build_web/index.html
#   ./build_web.sh serve    # build, then serve on http://localhost:8000
#
# The web build is single-threaded; everything it needs (config + obstacle/shape images + shaders +
# models) is baked into index.data and mounted at /app in the browser's virtual filesystem. Scenario
# generation and the obstacle/shape images are pure C++ (no OpenCV, no python scripts).
set -euo pipefail

EMSDK_DIR="${EMSDK:-$HOME/emsdk}"
if [ ! -f "$EMSDK_DIR/emsdk_env.sh" ]; then
    echo "Emscripten SDK not found at $EMSDK_DIR. Install it (see the link above) or set EMSDK." >&2
    exit 1
fi
# shellcheck disable=SC1091
source "$EMSDK_DIR/emsdk_env.sh" >/dev/null

emcmake cmake -S . -B build_web -DCMAKE_BUILD_TYPE=Release
cmake --build build_web --target dancers -j

echo
echo "Built build_web/index.html (+ .js / .wasm / .data)."
echo "To deploy, copy these four files (keep them together) to your webpage:"
echo "    cp build_web/index.{html,js,wasm,data} <directory/on/website>"
if [ "${1:-}" = "serve" ]; then
    echo "Serving on http://localhost:8000/  (Ctrl-C to stop)"
    emrun --no_browser --port 8000 build_web/index.html
else
    echo "Serve it with:  emrun build_web/index.html"
    echo "          (or)  python3 -m http.server -d build_web 8000   then open http://localhost:8000/"
fi
