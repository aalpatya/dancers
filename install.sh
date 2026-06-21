#!/usr/bin/env bash
# Source this script (`source install.sh`) to keep the compiler env exports below in your shell.
# Only enable errexit when executed directly - otherwise `set -e` would leak into your interactive
# shell and exit it on the next failing command.
(return 0 2>/dev/null) && SOURCED=1 || SOURCED=0
[[ $SOURCED -eq 0 ]] && set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Eigen, smooth and raylib are fetched automatically by CMake (FetchContent) - nothing else to install.

# ── macOS compiler flags (LLVM + OpenMP via Homebrew) ─────────────────────────
if [[ "$(uname)" == "Darwin" ]]; then
    echo "Detected macOS, setting up LLVM/OpenMP compiler flags..."
    export PATH="/opt/homebrew/opt/llvm/bin:$PATH"
    export LDFLAGS="$LDFLAGS -L/opt/homebrew/opt/llvm/lib -L/opt/homebrew/opt/libomp/lib"
    export CPPFLAGS="$CPPFLAGS -I/opt/homebrew/opt/llvm/include -I/opt/homebrew/opt/libomp/include"
    export CC="/opt/homebrew/opt/llvm/bin/clang"
    export CXX="/opt/homebrew/opt/llvm/bin/clang++"
fi

# ── CMake build ───────────────────────────────────────────────────────────────
echo "Configuring with CMake..."
mkdir -p build
cmake -S . -B build

echo "Building..."
cmake --build build -j16

# ── Done ──────────────────────────────────────────────────────────────────────
echo ""
echo "Build complete. Executables:"
echo ""
echo "    ./build/dancers [--cfg/-c config/*.yaml]          # the DANCeRS simulator"
echo ""
echo "    ./build/helpers/shape_editor                      # author shape-formation images"
echo "    ./build/examples/                                 # standalone GBP examples"
echo "              - gbp-three-variable-factorgraph        # demo of a three-variable chain"
echo "              - gbp-two-layer-factorgraph             # demo of multiple layers in a factorgraph"
echo "              - gbp-1d-line-fitting                   # example recreated from gaussianbp.github.io"
echo ""