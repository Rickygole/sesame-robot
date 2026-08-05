#!/usr/bin/env bash
# Build build/Sesame Voice.app -- a real macOS app bundle wrapping the
# companion's voice entrypoint, so Speech/AVFoundation get a proper
# NSBundle with an Info.plist instead of crashing with an uncatchable
# SIGABRT the instant they touch a privacy-sensitive API.
#
# Idempotent and safe to rerun: every step overwrites the same fixed
# output path (build/Sesame Voice.app), so re-running this after an edit
# to Info.plist, requirements.txt, or the venv just rebuilds it in place.
# The path staying fixed matters beyond convenience -- macOS's TCC grant
# for the mic/Speech permission is keyed to this app, so a path that
# moved around would mean re-granting permission on every rebuild.
#
#     tools/companion/app/build_app.sh
#     (normally invoked via `make app`, not directly)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPANION_DIR="$(dirname "$HERE")"
BUILD_DIR="$COMPANION_DIR/build"
APP_DIR="$BUILD_DIR/Sesame Voice.app"
VENV_DIR="$COMPANION_DIR/.venv"
BUNDLE_ID="com.sesame.voice"
EXE_NAME="SesameVoice"

log() { echo "[build_app] $*"; }
die() { echo "[build_app] ERROR: $*" >&2; exit 1; }

# --- 1. A modern Python via uv. System python3 (3.9 here) cannot run ---
#        pyobjc-framework-Speech; uv can fetch 3.12 with no brew, no sudo.
command -v uv >/dev/null 2>&1 || die \
  "uv is required (https://astral.sh/uv) -- 'brew install uv' or see their installer."

if [ ! -x "$VENV_DIR/bin/python3.12" ]; then
  log "creating venv at $VENV_DIR (python 3.12 via uv)"
  uv venv "$VENV_DIR" --python 3.12
fi

log "installing pyobjc into the venv (fast no-op if already satisfied)"
uv pip install --python "$VENV_DIR/bin/python3.12" -r "$COMPANION_DIR/requirements.txt"

VENV_PY="$VENV_DIR/bin/python3.12"
REAL_PY="$(/usr/bin/python3 -c "import os,sys; print(os.path.realpath(sys.argv[1]))" "$VENV_PY")"
[ -f "$REAL_PY" ] || die "could not resolve the real python3.12 binary behind $VENV_PY"

SITE_PACKAGES="$("$VENV_PY" -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")"
PYTHON_HOME="$("$VENV_PY" -c "import sys; print(sys.base_prefix)")"

log "python3.12  -> $REAL_PY"
log "site-packages -> $SITE_PACKAGES"
log "PYTHONHOME    -> $PYTHON_HOME"

# --- 2. Lay out the bundle. -------------------------------------------
mkdir -p "$APP_DIR/Contents/MacOS"

# A COPY, not a symlink: codesign refuses to sign a bundle whose main
# executable is a symlink ("must be a regular file"). Copying loses the
# interpreter's ability to find its own stdlib relative to argv0 (it was
# built expecting to live under .../bin/python3.12 with lib/python3.12
# as a sibling one level up) -- LSEnvironment below (PYTHONHOME /
# PYTHONPATH) is what fixes that back up rather than a symlink would.
cp "$REAL_PY" "$APP_DIR/Contents/MacOS/$EXE_NAME"
chmod +x "$APP_DIR/Contents/MacOS/$EXE_NAME"

# Some Python distributions (the python.org / Xcode framework builds)
# link the interpreter against Python3.framework via a relative
# @executable_path/../../../../Python3 load command that only resolves
# from its ORIGINAL location. If that's what we copied, fix the load
# path to the framework's real, absolute location so dyld can still
# find it once the binary has been relocated into the bundle. This is a
# no-op (the loop finds nothing) for the statically-linked interpreter
# that a plain `uv python install` gives you today, so it costs nothing
# to leave in for whichever Python build ends up installed.
while read -r dep; do
  [ -z "$dep" ] && continue
  rel="${dep#@executable_path/}"
  abs="$(cd "$(dirname "$REAL_PY")" && cd "$(dirname "$rel")" 2>/dev/null && pwd)/$(basename "$rel")" || continue
  log "fixing framework load path: $dep -> $abs"
  install_name_tool -change "$dep" "$abs" "$APP_DIR/Contents/MacOS/$EXE_NAME"
done < <(otool -L "$APP_DIR/Contents/MacOS/$EXE_NAME" \
            | awk '/@executable_path.*Python3[^.]*\(/{print $1}')

# --- 3. Info.plist, filled in from the template. -----------------------
sed \
  -e "s#@@PYTHON_HOME@@#$PYTHON_HOME#g" \
  -e "s#@@PYTHON_SITE_PACKAGES@@#$SITE_PACKAGES#g" \
  "$HERE/Info.plist" > "$APP_DIR/Contents/Info.plist"

# --- 4. Sign the WHOLE bundle, ad hoc, with a fixed identifier. --------
#
# The copied binary already carries its own ad-hoc, linker-signed
# signature from however it was built -- but that signature has no idea
# it now lives inside a bundle, so `codesign -dv` reports "Info.plist=
# not bound" and TCC's identity check for the mic/Speech grant fails
# before main() ever runs (observed as the process being silently
# killed within ~200ms of launch, no crash log). Re-signing the whole
# bundle with --force binds this exact Info.plist to this exact
# executable under one identifier, which is what TCC actually checks.
codesign -s - --force --identifier "$BUNDLE_ID" "$APP_DIR"

log "built: $APP_DIR"
log "run it with: make voice-listen  (NOT by launching Contents/MacOS/$EXE_NAME"
log "  directly -- see tools/companion/README.md for why 'open' is required)"
