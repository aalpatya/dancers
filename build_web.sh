#!/usr/bin/env bash
# Build the WebAssembly (browser) version of dancers.
#
# Requires the Emscripten SDK: https://emscripten.org/docs/getting_started/downloads.html
# Point EMSDK at your emsdk checkout, or it defaults to ~/emsdk.
#
# Usage:
#   ./build_web.sh          # configure + build -> build_web/index.html (front page) + the demos
#   ./build_web.sh serve    # build, then serve on http://localhost:8000
#
# index.html is the front page (the demos hub). It embeds, in an iframe:
#   - dancers.html  : the multirobot DANCeRS app (loaded with ?config=... per scenario)
#   - gbp-*.html    : the standalone visual GBP examples
# The web build is single-threaded; everything the DANCeRS app needs (config + obstacle/shape images +
# shaders + models) is baked into dancers.data and mounted at /app in the browser's virtual filesystem.
# Scenario generation and the obstacle/shape images are pure C++ (no OpenCV, no python scripts).
set -euo pipefail

EMSDK_DIR="${EMSDK:-$HOME/emsdk}"
if [ ! -f "$EMSDK_DIR/emsdk_env.sh" ]; then
    echo "Emscripten SDK not found at $EMSDK_DIR. Install it (see the link above) or set EMSDK." >&2
    exit 1
fi
# shellcheck disable=SC1091
source "$EMSDK_DIR/emsdk_env.sh" >/dev/null

emcmake cmake -S . -B build_web -DCMAKE_BUILD_TYPE=Release
cmake --build build_web --target dancers gbp-1d-line-fitting gbp-se2-localisation -j
# Drop any stale outputs from an earlier layout (when the app was emitted as index.*).
rm -f build_web/index.js build_web/index.wasm build_web/index.data

echo
echo "Built the web demos in build_web/."
echo "To deploy, copy these to your webpage (keep each .html with its sidecar files):"
echo "    cp build_web/index.html                          <dir>   # front page (the hub)"
echo "    cp build_web/dancers.{html,js,wasm,data}         <dir>   # multirobot app"
echo "    cp build_web/gbp-se2-localisation.{html,js,wasm} <dir>"
echo "    cp build_web/gbp-1d-line-fitting.{html,js,wasm}  <dir>"
if [ "${1:-}" = "serve" ]; then
    echo "Serving on http://localhost:8000/  (Ctrl-C to stop)"
    emrun --no_browser --port 8000 build_web/index.html
else
    echo "Serve it with:  emrun build_web/index.html"
    echo "          (or)  python3 -m http.server -d build_web 8000   then open http://localhost:8000/"
fi
