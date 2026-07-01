#!/bin/bash
set -e

if ! command -v cmake >/dev/null 2>&1; then
    echo "Missing cmake. Install with: brew install cmake"
    exit 1
fi

if [ -z "${PICO_TOOLCHAIN_PATH:-}" ]; then
    for candidate in /Applications/ArmGNUToolchain/*/arm-none-eabi; do
        if [ -x "${candidate}/bin/arm-none-eabi-gcc" ]; then
            export PICO_TOOLCHAIN_PATH="${candidate}"
            break
        fi
    done
fi

GCC_BIN="${PICO_TOOLCHAIN_PATH:+${PICO_TOOLCHAIN_PATH}/bin/}arm-none-eabi-gcc"
if ! command -v "${GCC_BIN}" >/dev/null 2>&1; then
    echo "Missing arm-none-eabi-gcc. Install the complete toolchain with: brew install --cask gcc-arm-embedded"
    exit 1
fi

NOSYS_SPECS="$(${GCC_BIN} -print-file-name=nosys.specs || true)"
if [ "${NOSYS_SPECS}" = "nosys.specs" ] || [ ! -f "${NOSYS_SPECS}" ]; then
    echo "arm-none-eabi-gcc is present, but it is missing nosys.specs/newlib."
    echo "Install the complete ARM toolchain with: brew install --cask gcc-arm-embedded"
    echo "Then rerun ./build.sh."
    exit 1
fi

TOOLCHAIN_ARGS=()
if [ -n "${PICO_TOOLCHAIN_PATH:-}" ]; then
    TOOLCHAIN_ARGS=("-DPICO_TOOLCHAIN_PATH=${PICO_TOOLCHAIN_PATH}")
fi

# rp2040
if [ -n "${PICO_SDK_PATH:-}" ]; then
    SDK_ARGS=("-DPICO_SDK_PATH=${PICO_SDK_PATH}")
else
    SDK_ARGS=("-DPICO_SDK_FETCH_FROM_GIT=ON")
fi

cmake -B build2040 -UPICO_SDK_FETCH_FROM_GIT_TAG -DPICO_BOARD=pico "${SDK_ARGS[@]}" "${TOOLCHAIN_ARGS[@]}"
cmake --build build2040 -j4
cp build2040/pic0rick-tx-short-mux.uf2 tx_short_extra_gnd_rp2040.uf2
