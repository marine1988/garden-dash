#!/usr/bin/env bash
#
# KAJO-Dash installer and flasher for macOS.
#
# The project's own launchers (kajo.bat, scripts/*.bat) are Windows-only. This
# script does the same job with pio directly: it installs PlatformIO if it is
# missing, checks out the firmware, builds it, and flashes it to the CYD over
# USB. Nothing here needs a signed release or a signing key — a build from your
# own checkout is meant to go over the cable.
#
# Usage:
#   scripts/install_macos.sh                 install, build, flash
#   scripts/install_macos.sh --no-flash      build only
#   scripts/install_macos.sh --port /dev/cu.wchusbserial-1230
#   scripts/install_macos.sh --monitor       open the serial monitor after flashing
#   scripts/install_macos.sh --dry-run       print the plan, change nothing
#
# Written for macOS's stock bash 3.2: no associative arrays, no ${var,,}, no
# mapfile.

set -euo pipefail

REPO_DEFAULT="https://github.com/marine1988/kajo-dash.git"
DIR_DEFAULT="./kajo-dash"
ENV_NAME="kajo"
MONITOR_SPEED="115200"

REPO="$REPO_DEFAULT"
DIR="$DIR_DEFAULT"
DIR_WAS_SET=0
PORT=""
DO_FLASH=1
DO_MONITOR=0
DRY_RUN=0
YES=0

# ── output ───────────────────────────────────────────────────────────────────

if [ -t 1 ] && [ -z "${NO_COLOR:-}" ]; then
  BOLD=$(printf '\033[1m'); DIM=$(printf '\033[2m')
  RED=$(printf '\033[31m'); GREEN=$(printf '\033[32m'); YELLOW=$(printf '\033[33m')
  RESET=$(printf '\033[0m')
else
  BOLD=""; DIM=""; RED=""; GREEN=""; YELLOW=""; RESET=""
fi

step() { printf '%s==>%s %s\n' "$BOLD" "$RESET" "$1"; }
info() { printf '    %s\n' "$1"; }
warn() { printf '%s !  %s%s\n' "$YELLOW" "$1" "$RESET" >&2; }
die()  { printf '%serror:%s %s\n' "$RED" "$RESET" "$1" >&2; exit 1; }

# Runs a command, or only prints it under --dry-run.
run() {
  if [ "$DRY_RUN" -eq 1 ]; then
    printf '    %s[dry-run]%s %s\n' "$DIM" "$RESET" "$*"
    return 0
  fi
  "$@"
}

usage() {
  cat <<'EOF'
KAJO-Dash installer and flasher for macOS.

Usage:
  scripts/install_macos.sh                 install, build, flash
  scripts/install_macos.sh --no-flash      build only
  scripts/install_macos.sh --port /dev/cu.wchusbserial-1230
  scripts/install_macos.sh --monitor       open the serial monitor after flashing
  scripts/install_macos.sh --dry-run       print the plan, change nothing

Options:
  --port DEV     serial device to flash (found automatically otherwise)
  --dir PATH     checkout location (default ./kajo-dash)
  --repo URL     git remote to clone (default the fork)
  --no-flash     build only
  --monitor      open the serial monitor at 115200 baud after flashing
  --dry-run      print the plan without installing, building or flashing
  --yes, -y      do not ask before installing platformio
  -h, --help     this text
EOF
}

# ── arguments ────────────────────────────────────────────────────────────────

while [ $# -gt 0 ]; do
  case "$1" in
    --port) PORT="${2:-}"; [ -n "$PORT" ] || die "--port needs a device path"; shift 2 ;;
    --port=*) PORT="${1#*=}"; shift ;;
    --dir) DIR="${2:-}"; [ -n "$DIR" ] || die "--dir needs a path"; DIR_WAS_SET=1; shift 2 ;;
    --dir=*) DIR="${1#*=}"; DIR_WAS_SET=1; shift ;;
    --repo) REPO="${2:-}"; [ -n "$REPO" ] || die "--repo needs a URL"; shift 2 ;;
    --repo=*) REPO="${1#*=}"; shift ;;
    --no-flash) DO_FLASH=0; shift ;;
    --monitor) DO_MONITOR=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    --yes|-y) YES=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1 (try --help)" ;;
  esac
done

if [ "$(uname -s)" != "Darwin" ]; then
  warn "This installer targets macOS; on $(uname -s) the serial-port names differ (it still builds)."
fi

# ── 1. PlatformIO ────────────────────────────────────────────────────────────

# pio lands somewhere different depending on how it was installed, and none of
# them are on PATH in a fresh shell. Add every likely location before deciding
# it is missing.
add_pio_paths() {
  PATH="$HOME/.local/bin:$PATH"
  PATH="$PATH:/opt/homebrew/bin:/usr/local/bin"
  for pydir in "$HOME"/Library/Python/*/bin; do
    if [ -d "$pydir" ]; then
      PATH="$PATH:$pydir"
    fi
  done
  export PATH
}

find_pio() { command -v pio 2>/dev/null || true; }

# The Python PlatformIO runs its tools with. On a Homebrew or pipx install this
# is the launcher's own virtualenv, not any system python.
pio_python() {
  "$PIO" system info 2>/dev/null | sed -n 's/^Python Executable *//p' | head -1
}

# PlatformIO builds esptool from a downloaded package whose own post-install
# script installs its Python deps (intelhex among them) into the package's
# _contrib directory. In the field that post-install is sometimes skipped, and
# recent esptool imports intelhex unconditionally at module load, so an ESP32
# build dies at bootloader.bin with "No module named 'intelhex'". Make sure the
# module is importable by the Python PlatformIO runs, and by the legacy penv if
# one exists.
ensure_python_module() {
  local mod="$1" py
  for py in "$(pio_python)" "$HOME/.platformio/penv/bin/python"; do
    [ -n "$py" ] || continue
    [ -x "$py" ] || continue
    if "$py" -c "import $mod" >/dev/null 2>&1; then
      continue
    fi
    info "installing the '$mod' module PlatformIO's tools need"
    if ! "$py" -m pip install --quiet "$mod" >/dev/null 2>&1; then
      # A uv/pipx venv has no pip of its own until ensurepip bootstraps it.
      "$py" -m ensurepip --upgrade >/dev/null 2>&1 || true
      "$py" -m pip install --quiet "$mod" >/dev/null 2>&1 ||
        warn "could not install '$mod' into $py"
    fi
  done
}

add_pio_paths
PIO=$(find_pio)

if [ -z "$PIO" ]; then
  step "PlatformIO is not installed; installing it"
  if command -v uv >/dev/null 2>&1; then
    info "using uv"
    run uv tool install platformio
  elif command -v brew >/dev/null 2>&1; then
    info "using Homebrew"
    run brew install platformio
  elif command -v pipx >/dev/null 2>&1; then
    info "using pipx"
    run pipx install platformio
  elif command -v python3 >/dev/null 2>&1; then
    info "using pip3 --user (Homebrew's Python needs --break-system-packages)"
    if [ "$DRY_RUN" -eq 0 ]; then
      python3 -m pip install --user platformio 2>/dev/null ||
        python3 -m pip install --user --break-system-packages platformio ||
        die "could not install PlatformIO. Install uv (https://docs.astral.sh/uv/) or Homebrew and re-run."
    else
      printf '    %s[dry-run]%s python3 -m pip install --user platformio\n' "$DIM" "$RESET"
    fi
  else
    die "no way to install Python packages found. Install uv (https://docs.astral.sh/uv/) then re-run."
  fi
  add_pio_paths
  PIO=$(find_pio)
  [ -n "$PIO" ] || die "PlatformIO installed but not on PATH. Open a new terminal and re-run."
else
  step "PlatformIO found: $PIO"
  info "$("$PIO" --version 2>/dev/null || echo 'version unknown')"
fi

# ── 2. Checkout ──────────────────────────────────────────────────────────────

# Run from inside a checkout and it builds that one instead of cloning a nested
# copy; running ./scripts/install_macos.sh is exactly this case.
if [ "$DIR_WAS_SET" -eq 0 ] && [ -f "./platformio.ini" ] && [ -d "./src/lvgl_app" ]; then
  DIR="."
fi

if [ -d "$DIR/.git" ]; then
  step "Using existing checkout at $DIR"
  if [ -n "$(git -C "$DIR" status --porcelain 2>/dev/null)" ]; then
    warn "the checkout has local changes; leaving it as it is"
  fi
elif [ -e "$DIR" ]; then
  die "$DIR exists but is not a git checkout"
else
  step "Cloning $REPO into $DIR"
  run git clone "$REPO" "$DIR"
fi

[ "$DRY_RUN" -eq 1 ] || [ -d "$DIR" ] || die "checkout at $DIR is missing"

# ── 3. Serial port ───────────────────────────────────────────────────────────

# Candidate names by driver: CH340/CH9102 (wchusbserial), CP210x (SLAB_USBtoUART),
# FTDI and the ESP32 ROM's own (usbserial, usbmodem). The CYD ships with a CH340.
list_ports() {
  if [ "$(uname -s)" = "Darwin" ]; then
    ls /dev/cu.usbserial* /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART* \
       /dev/cu.usbmodem* 2>/dev/null || true
  else
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
  fi
}

if [ "$DO_FLASH" -eq 1 ] && [ -z "$PORT" ]; then
  step "Looking for the display on USB"
  PORTS=$(list_ports)
  COUNT=$(printf '%s\n' "$PORTS" | grep -c . || true)
  if [ "$COUNT" -eq 0 ]; then
    warn "no serial device found."
    if [ "$(uname -s)" = "Darwin" ]; then
      info "plug the CYD in with a USB DATA cable (charge-only cables will not work),"
      info "then check 'pio device list'. If the port still does not appear, install"
      info "the USB-serial driver for the board's chip:"
      info "  CH340/CH9102 (most CYDs): https://www.wch-ic.com/downloads/CH34XSER_MAC_ZIP.html"
      info "  CP2102:                   https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers"
    fi
    info "building anyway; flashing needs a port (re-run with --port, or just plug it in)."
    DO_FLASH=0
  elif [ "$COUNT" -eq 1 ]; then
    PORT=$(printf '%s\n' "$PORTS" | head -1)
    info "using $PORT"
  else
    warn "more than one serial device is connected:"
    printf '%s\n' "$PORTS" | while IFS= read -r p; do
      if [ -n "$p" ]; then info "$p"; fi
    done
    die "choose one with --port <device>"
  fi
fi

# ── 4. Build ─────────────────────────────────────────────────────────────────

step "Building the firmware (first run downloads the ESP32 toolchain, ~2 GB, and takes a few minutes)"
info "environment: $ENV_NAME"

build_firmware() { "$PIO" run -e "$ENV_NAME" --project-dir "$DIR"; }

if [ "$DRY_RUN" -eq 1 ]; then
  printf '    %s[dry-run]%s %s run -e %s --project-dir %s\n' "$DIM" "$RESET" "$PIO" "$ENV_NAME" "$DIR"
else
  # Recent esptool imports intelhex unconditionally, and PlatformIO's own copy
  # of it is not always complete. Make sure it is importable before the build
  # so a first run does not have to fail to reveal it.
  ensure_python_module intelhex
  BUILD_LOG=$(mktemp)
  if build_firmware 2>&1 | tee "$BUILD_LOG"; then
    rm -f "$BUILD_LOG"
  elif grep -q "intelhex" "$BUILD_LOG"; then
    # Installing intelhex into PlatformIO's python did not stick. Its own copy
    # lives inside the downloaded esptool package; re-fetching that package
    # re-runs the post-install that populates it.
    warn "esptool still cannot find intelhex; re-fetching its package"
    rm -f "$BUILD_LOG"
    rm -rf "$HOME/.platformio/packages/tool-esptoolpy"
    ensure_python_module intelhex
    build_firmware || die "build still failed after refreshing esptool (see the output above)"
  else
    rm -f "$BUILD_LOG"
    die "build failed (see the output above)"
  fi
fi

# ── 5. Flash ─────────────────────────────────────────────────────────────────

flash_failed_help() {
  cat >&2 <<'EOF'

If the upload failed with "Wrong boot mode detected" or "Failed to connect":
  1. Hold the BOOT button on the CYD.
  2. Tap RST (or EN) once, still holding BOOT.
  3. Keep holding BOOT until "Connecting..." appears, then release.
  4. Re-run this script.
A charge-only USB cable is the other common cause; use a data cable.
EOF
}

if [ "$DO_FLASH" -eq 1 ]; then
  step "Flashing to $PORT"
  info "this keeps your saved settings"
  if [ "$DRY_RUN" -eq 1 ]; then
    printf '    %s[dry-run]%s %s run -e %s -t upload --upload-port %s --project-dir %s\n' \
      "$DIM" "$RESET" "$PIO" "$ENV_NAME" "$PORT" "$DIR"
  elif ! "$PIO" run -e "$ENV_NAME" -t upload --upload-port "$PORT" --project-dir "$DIR"; then
    flash_failed_help
    exit 1
  fi
  step "Done. The display restarts into KAJO-Dash."
else
  step "Skipped flashing. Build output: $DIR/.pio/build/$ENV_NAME/firmware.bin"
fi

# ── 6. Monitor (optional) ────────────────────────────────────────────────────

if [ "$DO_MONITOR" -eq 1 ]; then
  if [ -z "$PORT" ]; then
    die "--monitor needs a port (pass --port)"
  fi
  step "Opening the serial monitor at $MONITOR_SPEED baud (Ctrl-C to leave)"
  info "the firmware prints one LVGL workload line every 5 s"
  run "$PIO" device monitor --port "$PORT" --baud "$MONITOR_SPEED" --project-dir "$DIR"
fi
