#!/usr/bin/env bash
set -euo pipefail

# Complete, self-verifying offline build cache generator (v9 resume) for the
# Unni CO2 Sensor / esphome-i2c-sniffer project.
#
# Usage (from the directory containing esphome-i2c-sniffer):
#   chmod +x build-complete-esphome-offline-bundle.sh
#   ./build-complete-esphome-offline-bundle.sh
#
# Or from anywhere:
#   PROJECT_DIR=/absolute/path/to/esphome-i2c-sniffer \
#     ./build-complete-esphome-offline-bundle.sh

ESPHOME_VERSION="${ESPHOME_VERSION:-2026.7.4}"
TARGET_PLATFORM="${TARGET_PLATFORM:-linux/amd64}"
PROJECT_DIR="${PROJECT_DIR:-$PWD/esphome-i2c-sniffer}"
PROJECT_DIR="$(cd "$PROJECT_DIR" 2>/dev/null && pwd)" || {
  echo "ERROR: project directory not found: ${PROJECT_DIR}" >&2
  exit 2
}

BASE_DIR="${BUNDLE_BASE_DIR:-$PWD}"
OUT_DIR="$BASE_DIR/offline-bundle"
ARCHIVE="$BASE_DIR/esphome-full-offline-cache-${ESPHOME_VERSION}-linux-amd64.zip"
IMAGE="esphome-offline-builder:${ESPHOME_VERSION}"
DOCKERFILE="$OUT_DIR/Dockerfile.builder"
PRIME_SCRIPT="$OUT_DIR/prime-inside-container.sh"
VERIFY_SCRIPT="$OUT_DIR/verify-inside-container.sh"
REGISTRY_WRAPPER="$OUT_DIR/esphome-registry-lock-wrapper.py"

REAL_CONFIGS=(
  i2c-sniffer.yaml
  i2c-sniffer-debug.yaml
  i2c-sniffer-no-ble.yaml
  i2c-sniffer-ble-only.yaml
  i2c-sniffer-sht43-probe.yaml
)

for cfg in "${REAL_CONFIGS[@]}"; do
  if [[ ! -f "$PROJECT_DIR/$cfg" ]]; then
    echo "ERROR: missing project config: $PROJECT_DIR/$cfg" >&2
    exit 2
  fi
done

if [[ ! -f "$PROJECT_DIR/tests/host/run_host_tests.py" ]]; then
  echo "ERROR: host test runner not found: $PROJECT_DIR/tests/host/run_host_tests.py" >&2
  exit 2
fi

command -v docker >/dev/null 2>&1 || {
  echo "ERROR: docker is not installed or not in PATH" >&2
  exit 2
}

mkdir -p "$OUT_DIR"
rm -f "$ARCHIVE"

echo "Using resumable bundle directory: $OUT_DIR"
if [[ -f "$OUT_DIR/PRIME_SUCCESS" ]]; then
  echo "Existing primed cache found; it will be reused and extended."
fi

cat > "$DOCKERFILE" <<'DOCKER'
FROM python:3.13-bookworm

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      clang \
      git \
      curl \
      ca-certificates \
      zip \
      unzip \
      xz-utils \
      bzip2 \
      pkg-config \
      file \
    && rm -rf /var/lib/apt/lists/*
DOCKER

cat > "$REGISTRY_WRAPPER" <<'PYWRAP'
#!/usr/bin/env python3
"""Run ESPHome with a record/replay lock for PlatformIO registry resolution.

ESPHome 2026.7.4 resolves registry library versions during code generation even
when the corresponding extracted library is already present in pio_components.
That normally requires api.registry.platformio.org.  This wrapper records the
resolved (owner, name, requirements) -> (owner, name, version, download_url)
tuples while online and replays them when building with no network.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys

from esphome.platformio import library as pio_library

MODE = os.environ.get("ESPHOME_REGISTRY_LOCK_MODE", "record")
LOCK_PATH = Path(os.environ.get("ESPHOME_REGISTRY_LOCK_FILE", "/out/platformio-registry-lock.json"))


def make_key(owner, pkgname, requirements):
    return json.dumps(
        [owner, pkgname, sorted(requirements)],
        separators=(",", ":"),
        ensure_ascii=True,
    )


def load_lock():
    if not LOCK_PATH.is_file():
        return {}
    data = json.loads(LOCK_PATH.read_text())
    if not isinstance(data, dict):
        raise RuntimeError(f"Invalid registry lock format: {LOCK_PATH}")
    return data


_original = pio_library._resolve_registry_version
_lock = load_lock()


def locked_resolve(owner, pkgname, requirements):
    key = make_key(owner, pkgname, requirements)
    if MODE == "replay":
        value = _lock.get(key)
        if value is None:
            raise RuntimeError(
                "Offline PlatformIO registry lock miss for "
                f"owner={owner!r} package={pkgname!r} requirements={sorted(requirements)!r}. "
                f"Refresh the offline bundle with network access."
            )
        if not isinstance(value, list) or len(value) != 4:
            raise RuntimeError(f"Invalid registry lock entry for {key}")
        return tuple(value)

    result = _original(owner, pkgname, requirements)
    _lock[key] = list(result)
    LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = LOCK_PATH.with_suffix(LOCK_PATH.suffix + ".tmp")
    tmp.write_text(json.dumps(_lock, indent=2, sort_keys=True) + "\n")
    tmp.replace(LOCK_PATH)
    return result


pio_library._resolve_registry_version = locked_resolve

from esphome.__main__ import main

# Make the downstream CLI see a normal executable name.
sys.argv[0] = "esphome"
raise SystemExit(main())
PYWRAP
chmod +x "$REGISTRY_WRAPPER"

cat > "$PRIME_SCRIPT" <<'PRIME'
#!/usr/bin/env bash
set -euo pipefail
set -x

ESPHOME_VERSION="${ESPHOME_VERSION:?}"

export HOME=/opt/home
export PLATFORMIO_CORE_DIR=/opt/platformio
export PIP_DISABLE_PIP_VERSION_CHECK=1
export ESPHOME_REGISTRY_LOCK_MODE=record
export ESPHOME_REGISTRY_LOCK_FILE=/out/platformio-registry-lock.json
ESPHOME_LOCKED=(/opt/esphome/bin/python /out/esphome-registry-lock-wrapper.py)
IDF_COMPONENT_MIRROR=/out/idf-component-mirror
BUILD_WORK=/tmp/esphome-project
mkdir -p "$HOME/.cache" "$PLATFORMIO_CORE_DIR"

echo '================================================================='
echo 'RESUME EXISTING CACHES'
echo '================================================================='

if [[ -d /out/esphome-cache ]]; then
  echo "Restoring existing ESPHome/ESP-IDF cache"
  mkdir -p "$HOME/.cache/esphome"
  cp -a /out/esphome-cache/. "$HOME/.cache/esphome/"
fi

if [[ -d /out/platformio ]]; then
  echo "Restoring existing PlatformIO cache"
  cp -a /out/platformio/. "$PLATFORMIO_CORE_DIR/"
fi


echo '================================================================='
echo 'ONLINE PRIME: INSTALL/RESTORE ESPHOME'
echo '================================================================='

rm -rf /opt/esphome
python -m venv /opt/esphome
if compgen -G "/out/wheelhouse/esphome-${ESPHOME_VERSION//./_}*.whl" >/dev/null || ls /out/wheelhouse/esphome-*.whl >/dev/null 2>&1; then
  echo "Installing ESPHome from existing wheelhouse first"
  if ! /opt/esphome/bin/python -m pip install --no-index --find-links=/out/wheelhouse "esphome==$ESPHOME_VERSION"; then
    echo "Existing wheelhouse incomplete; falling back to online install"
    /opt/esphome/bin/python -m pip install --upgrade pip setuptools wheel
    /opt/esphome/bin/python -m pip install "esphome==$ESPHOME_VERSION"
  fi
else
  /opt/esphome/bin/python -m pip install --upgrade pip setuptools wheel
  /opt/esphome/bin/python -m pip install "esphome==$ESPHOME_VERSION"
fi

/opt/esphome/bin/esphome version
/opt/esphome/bin/platformio --version
python --version
g++ --version | head -1

# Warm PlatformIO's native platform and SCons explicitly.
mkdir -p /tmp/pio-native-test/src
cat >/tmp/pio-native-test/platformio.ini <<'PIO'
[env:native]
platform = native
build_flags = -std=gnu++20
PIO
cat >/tmp/pio-native-test/src/main.cpp <<'CPP'
#include <concepts>
#include <cstdio>
int main() {
  static_assert(std::integral<int>);
  std::puts("native toolchain OK");
  return 0;
}
CPP
(
  cd /tmp/pio-native-test
  /opt/esphome/bin/platformio run
  .pio/build/native/program
)

echo '================================================================='
echo 'CREATE CONTAINER-LOCAL PROJECT WORKTREE'
echo '================================================================='
rm -rf "$BUILD_WORK"
mkdir -p "$BUILD_WORK"
# Build on the container filesystem, not the macOS Docker bind mount.  ESPHome
# switches all YAML variants through the same build directory and may need to
# recursively delete it between variants; Docker Desktop bind mounts can make
# shutil.rmtree() spuriously fail with ENOTEMPTY.
tar -C /work --exclude=.esphome -cf - . | tar -C "$BUILD_WORK" -xf -
mkdir -p "$BUILD_WORK/.esphome"
if [[ -d /out/pio-components-cache ]]; then
  echo "Restoring existing resolved PlatformIO libraries into local worktree"
  cp -a /out/pio-components-cache "$BUILD_WORK/.esphome/pio_components"
fi
cd "$BUILD_WORK"

REAL_CONFIGS=(
  i2c-sniffer.yaml
  i2c-sniffer-debug.yaml
  i2c-sniffer-no-ble.yaml
  i2c-sniffer-ble-only.yaml
  i2c-sniffer-sht43-probe.yaml
)

echo '================================================================='
echo 'ONLINE PRIME: REAL ESP32-C3 BUILD MATRIX + PER-VARIANT IDF LOCK CAPTURE'
echo '================================================================='

# All real YAML variants share the ESPHome node/build name `i2csniffer`.
# Therefore the next variant can clean the previous variant's generated IDF
# project, including dependencies.lock. Capture each variant BEFORE
# compiling the next one so the union of dependencies is not lost. Mirror creation is done later from the exact union of archived locks.
COMPOTE=""
IDF_LOCK_ARCHIVE=/out/idf-dependency-locks
mkdir -p "$IDF_COMPONENT_MIRROR" "$IDF_LOCK_ARCHIVE"

find_compote() {
  if [[ -n "$COMPOTE" && -x "$COMPOTE" ]]; then
    return 0
  fi
  while IFS= read -r candidate; do
    if [[ -x "$candidate" ]]; then
      COMPOTE="$candidate"
      break
    fi
  done < <(find "$HOME/.cache/esphome/idf/penvs" -type f -name compote -perm -111 2>/dev/null | sort)
  [[ -n "$COMPOTE" ]] || {
    echo "ERROR: compote was not found in the ESP-IDF Python environment" >&2
    exit 6
  }
}

for cfg in "${REAL_CONFIGS[@]}"; do
  safe_cfg="${cfg//[^A-Za-z0-9_.-]/_}"
  archived_lock="$IDF_LOCK_ARCHIVE/$safe_cfg.dependencies.lock"

  # Resume support: a previous interrupted run may already have captured the
  # exact solved dependency lock for this variant. Keep it and avoid repeating
  # an expensive online ESP32 build solely for mirror discovery. The final
  # --network none verification still rebuilds every variant from scratch.
  if [[ -s "$archived_lock" ]]; then
    echo
    echo "=== REAL ESP32 BUILD: $cfg ==="
    echo "Reusing archived IDF dependency lock: $archived_lock"
    continue
  fi

  echo
  echo "=== REAL ESP32 BUILD: $cfg ==="
  "${ESPHOME_LOCKED[@]}" compile "$cfg"

  build_root="$BUILD_WORK/.esphome/build/i2csniffer"
  lock_file="$build_root/dependencies.lock"
  if [[ ! -s "$lock_file" ]]; then
    echo "ERROR: no dependencies.lock after successful build of $cfg: $lock_file" >&2
    find "$BUILD_WORK/.esphome/build" -maxdepth 3 -name dependencies.lock -print >&2 || true
    exit 6
  fi

  cp -a "$lock_file" "$archived_lock"
  echo "Archived IDF dependency lock for $cfg"
done

find_compote
echo "Using IDF Component Manager CLI: $COMPOTE"
"$COMPOTE" --version || true

echo '================================================================='
echo 'ONLINE PRIME: HOST TEST MATRIX'
echo '================================================================='

/opt/esphome/bin/python tests/host/run_host_tests.py

echo '================================================================='
echo 'FINALIZE ESP-IDF COMPONENT-MANAGER OFFLINE MIRROR'
echo '================================================================='

# dependencies.lock contains the flattened dependency set selected by the IDF
# solver. Merge the archived lock from EVERY YAML variant and explicitly mirror
# every service-registry component at its solved version. This catches
# transitive dependencies (for example zorxx/multipart-parser) and dependencies
# that exist only in a variant which has already been cleaned from .esphome.
LOCK_COMPONENTS_FILE=/out/idf-component-lock-components.txt
/opt/esphome/bin/python - <<'PYLOCK'
from pathlib import Path
import yaml

root = Path('/out/idf-dependency-locks')
found = {}
lock_files = sorted(root.glob('*.dependencies.lock'))
if len(lock_files) < 5:
    raise SystemExit(f'ERROR: expected locks for 5 YAML variants, found {len(lock_files)} in {root}')

for lock_path in lock_files:
    data = yaml.safe_load(lock_path.read_text()) or {}
    deps = data.get('dependencies') or {}
    if not isinstance(deps, dict):
        continue
    for name, meta in deps.items():
        if name == 'idf' or not isinstance(meta, dict):
            continue
        source = meta.get('source') or {}
        if not isinstance(source, dict) or source.get('type') != 'service':
            continue
        version = meta.get('version')
        if not version:
            raise SystemExit(f'ERROR: registry dependency {name!r} in {lock_path} has no solved version')
        found.setdefault(name, set()).add(str(version))

resolved = [f'{name}=={version}' for name in sorted(found) for version in sorted(found[name])]
out = Path('/out/idf-component-lock-components.txt')
out.write_text(''.join(component + '\n' for component in resolved))
print(f'Found {len(resolved)} solved registry component versions across {len(lock_files)} variant lock files:')
for component in resolved:
    print(f'  {component}')
PYLOCK

while IFS= read -r component; do
  [[ -n "$component" ]] || continue
  echo "Explicitly mirroring solved IDF component: $component"
  "$COMPOTE" registry sync \
    --component "$component" \
    "$IDF_COMPONENT_MIRROR"
done < "$LOCK_COMPONENTS_FILE"

# A mirror without metadata/index JSON cannot resolve versions offline.
if ! find "$IDF_COMPONENT_MIRROR" -type f -name '*.json' -print -quit | grep -q .; then
  echo "ERROR: IDF component mirror contains no JSON metadata" >&2
  find "$IDF_COMPONENT_MIRROR" -maxdepth 4 -type f -print >&2 || true
  exit 6
fi

du -sh "$IDF_COMPONENT_MIRROR"

echo '================================================================='
echo 'CREATE COMPLETE PYTHON WHEELHOUSE'
echo '================================================================='

mkdir -p /out/wheelhouse

# First populate normal downloadable artifacts.
/opt/esphome/bin/python -m pip download \
  --dest /out/wheelhouse \
  setuptools wheel "esphome==$ESPHOME_VERSION"

# Then ensure every dependency is available as a wheel, including projects
# whose index release is only an sdist for this environment.
/opt/esphome/bin/python -m pip wheel \
  --wheel-dir /out/wheelhouse \
  setuptools wheel "esphome==$ESPHOME_VERSION"

# Remove source distributions when a corresponding wheel is present. Leaving
# them would not hurt, but a wheel-only bundle makes offline behavior clearer.
python - <<'PY'
from pathlib import Path
root = Path('/out/wheelhouse')
wheels = [p.name.lower().replace('-', '_') for p in root.glob('*.whl')]
for src in list(root.glob('*.tar.gz')) + list(root.glob('*.zip')):
    stem = src.name.lower().replace('-', '_')
    pkg = stem.split('_')[0]
    if any(w.startswith(pkg + '_') for w in wheels):
        try:
            src.unlink()
        except OSError:
            pass
PY

echo '================================================================='
echo 'SAVE WARMED CACHES'
echo '================================================================='

# PlatformIO platforms/packages/cache.
rm -rf /out/platformio
cp -a "$PLATFORMIO_CORE_DIR" /out/platformio

# ESPHome's complete cache. This is the crucial part that contains:
#   ~/.cache/esphome/idf/frameworks
#   ~/.cache/esphome/idf/tools
#   ~/.cache/esphome/idf/penvs
#   ~/.cache/esphome/idf/dist
# and any other ESPHome-managed caches.
rm -rf /out/esphome-cache
mkdir -p /out/esphome-cache
if [[ -d "$HOME/.cache/esphome" ]]; then
  cp -a "$HOME/.cache/esphome/." /out/esphome-cache/
fi

# Keep the warmed project build tree too. It is optional for correctness, but
# useful for diagnostics and incremental builds. Offline verification below
# deletes its build output, so it cannot mask a missing cache dependency.
rm -rf /out/esphome-build-cache
if [[ -d "$BUILD_WORK/.esphome" ]]; then
  cp -a "$BUILD_WORK/.esphome" /out/esphome-build-cache
fi

# Save the extracted/resolved PlatformIO libraries separately.  These are
# addressed by a hash of the registry download URL, so replaying the recorded
# registry resolution makes ESPHome reuse them without touching the network.
rm -rf /out/pio-components-cache
if [[ -d "$BUILD_WORK/.esphome/pio_components" ]]; then
  cp -a "$BUILD_WORK/.esphome/pio_components" /out/pio-components-cache
else
  echo "ERROR: warmed pio_components cache missing" >&2
  exit 5
fi

[[ -s /out/platformio-registry-lock.json ]] || {
  echo "ERROR: PlatformIO registry lock was not populated" >&2
  exit 5
}

/opt/esphome/bin/python -m pip freeze > /out/requirements-frozen.txt
/opt/esphome/bin/platformio pkg list --global > /out/platformio-global-packages.txt || true
/opt/esphome/bin/platformio system info > /out/platformio-system-info.txt || true

{
  echo 'ESPHome:'
  /opt/esphome/bin/esphome version
  echo
  echo 'PlatformIO:'
  /opt/esphome/bin/platformio --version
  echo
  echo 'Python:'
  python --version
  echo
  echo 'GCC:'
  g++ --version | head -1
  echo
  echo 'Clang:'
  clang++ --version | head -1
  echo
  echo 'Architecture:'
  uname -a
  echo
  echo 'ESPHome cache:'
  du -sh /out/esphome-cache || true
  echo
  echo 'PlatformIO cache:'
  du -sh /out/platformio || true
  echo
  echo 'IDF component mirror:'
  du -sh /out/idf-component-mirror || true
} > /out/environment.txt

date -u +'%Y-%m-%dT%H:%M:%SZ' > /out/PRIME_SUCCESS

echo '================================================================='
echo 'ONLINE PRIME COMPLETE'
echo '================================================================='
PRIME

cat > "$VERIFY_SCRIPT" <<'VERIFY'
#!/usr/bin/env bash
set -euo pipefail
set -x

ESPHOME_VERSION="${ESPHOME_VERSION:?}"

export HOME=/opt/home
export PLATFORMIO_CORE_DIR=/opt/platformio
export PIP_NO_INDEX=1
export PIP_DISABLE_PIP_VERSION_CHECK=1
export ESPHOME_REGISTRY_LOCK_MODE=replay
export ESPHOME_REGISTRY_LOCK_FILE=/bundle/platformio-registry-lock.json
ESPHOME_LOCKED=(/opt/esphome/bin/python /bundle/esphome-registry-lock-wrapper.py)
BUILD_WORK=/tmp/esphome-project

# Official IDF Component Manager offline source. The local mirror is checked
# before any registry/storage URL, so all required components resolve without
# network access.
export IDF_COMPONENT_LOCAL_STORAGE_URL=file:///bundle/idf-component-mirror

rm -rf "$HOME" "$PLATFORMIO_CORE_DIR" /opt/esphome
mkdir -p "$HOME/.cache/esphome" "$PLATFORMIO_CORE_DIR" "$HOME/.espressif"

# Restore cache CONTENTS to the exact paths used during priming.  Do not copy
# the esphome-cache directory itself into an already existing esphome directory:
# that would create ~/.cache/esphome/esphome-cache/idf and make ESPHome think
# the complete IDF cache is missing.
cp -a /bundle/esphome-cache/. "$HOME/.cache/esphome/"
cp -a /bundle/platformio/. "$PLATFORMIO_CORE_DIR/"

# Configure the official ESP-IDF Component Manager local mirror *after* cache
# restoration so a restored cache can never hide/replace the offline config.
cat > "$HOME/.espressif/idf_component_manager.yml" <<'IDFCFG'
profiles:
  default:
    local_storage_url:
      - file:///bundle/idf-component-mirror
IDFCFG
mkdir -p "$HOME/.cache/esphome/idf"
cp "$HOME/.espressif/idf_component_manager.yml" \
  "$HOME/.cache/esphome/idf/idf_component_manager.yml"

# Fail immediately if the supposedly complete IDF cache was restored at the
# wrong level or is incomplete.  This is deliberately before any ESPHome build,
# so missing cache content is diagnosed in seconds rather than after codegen.
IDF_ROOT="$HOME/.cache/esphome/idf"
echo '================================================================='
echo 'OFFLINE CACHE PREFLIGHT'
echo '================================================================='
[[ -d "$IDF_ROOT/frameworks/5.5.5" ]] || {
  echo "ERROR: restored ESP-IDF framework missing: $IDF_ROOT/frameworks/5.5.5" >&2
  find "$HOME/.cache/esphome" -maxdepth 4 -type d -print >&2 || true
  exit 7
}
[[ -x "$IDF_ROOT/tools/riscv32-esp-elf/esp-14.2.0_20260121/riscv32-esp-elf/bin/riscv32-esp-elf-g++" ]] || {
  echo "ERROR: restored ESP32-C3 compiler missing from IDF tools cache" >&2
  exit 7
}
[[ -x "$IDF_ROOT/penvs/5.5.5/bin/python" ]] || {
  echo "ERROR: restored ESP-IDF Python environment missing" >&2
  exit 7
}
[[ -d "$IDF_ROOT/dist" ]] || {
  echo "ERROR: restored ESP-IDF dist cache missing" >&2
  exit 7
}
echo "ESP-IDF framework: OK"
echo "ESP32-C3 compiler: OK"
echo "ESP-IDF Python env: OK"
echo "ESP-IDF dist cache: OK"
du -sh "$IDF_ROOT" || true

# Install ESPHome into a brand-new venv using *only* the wheelhouse.
python -m venv /opt/esphome
/opt/esphome/bin/python -m pip install \
  --no-index \
  --find-links=/bundle/wheelhouse \
  "esphome==$ESPHOME_VERSION"

/opt/esphome/bin/esphome version
/opt/esphome/bin/platformio --version

echo '================================================================='
echo 'CREATE CLEAN CONTAINER-LOCAL PROJECT WORKTREE'
echo '================================================================='
rm -rf "$BUILD_WORK"
mkdir -p "$BUILD_WORK"
tar -C /work --exclude=.esphome -cf - . | tar -C "$BUILD_WORK" -xf -
# Start from a clean ESPHome project state, then restore only the portable
# resolved-library cache. This also keeps all build/delete activity off the
# macOS Docker bind mount.
mkdir -p "$BUILD_WORK/.esphome"
cp -a /bundle/pio-components-cache "$BUILD_WORK/.esphome/pio_components"
cd "$BUILD_WORK"

REAL_CONFIGS=(
  i2c-sniffer.yaml
  i2c-sniffer-debug.yaml
  i2c-sniffer-no-ble.yaml
  i2c-sniffer-ble-only.yaml
  i2c-sniffer-sht43-probe.yaml
)

echo '================================================================='
echo 'NETWORK-OFF VERIFICATION: FRESH ESP32-C3 BUILD MATRIX'
echo '================================================================='

for cfg in "${REAL_CONFIGS[@]}"; do
  echo
  echo "=== OFFLINE REAL ESP32 BUILD: $cfg ==="
  "${ESPHOME_LOCKED[@]}" compile "$cfg"
done

echo '================================================================='
echo 'NETWORK-OFF VERIFICATION: HOST TEST MATRIX'
echo '================================================================='

/opt/esphome/bin/python tests/host/run_host_tests.py

echo '================================================================='
echo 'NETWORK-OFF VERIFICATION PASSED'
echo '================================================================='

date -u +'%Y-%m-%dT%H:%M:%SZ' > /verify-result/OFFLINE_VERIFICATION_SUCCESS
VERIFY

chmod +x "$PRIME_SCRIPT" "$VERIFY_SCRIPT"

echo "Project:          $PROJECT_DIR"
echo "ESPHome:          $ESPHOME_VERSION"
echo "Docker platform:  $TARGET_PLATFORM"
echo "Builder image:    $IMAGE"
echo "Bundle directory: $OUT_DIR"
echo "Archive:          $ARCHIVE"
echo

echo '================================================================='
echo '1/4 BUILDING HELPER IMAGE'
echo '================================================================='
docker build \
  --platform "$TARGET_PLATFORM" \
  -f "$DOCKERFILE" \
  -t "$IMAGE" \
  "$OUT_DIR"

echo '================================================================='
echo '2/4 PRIMING ALL CACHES WITH NETWORK ACCESS'
echo '================================================================='
docker run --rm \
  --platform "$TARGET_PLATFORM" \
  -e "ESPHOME_VERSION=$ESPHOME_VERSION" \
  -v "$PROJECT_DIR:/work" \
  -v "$OUT_DIR:/out" \
  -w /work \
  "$IMAGE" \
  bash /out/prime-inside-container.sh

[[ -s "$OUT_DIR/PRIME_SUCCESS" ]] || {
  echo 'ERROR: priming container did not create PRIME_SUCCESS' >&2
  exit 3
}
[[ -d "$OUT_DIR/wheelhouse" ]] || { echo 'ERROR: wheelhouse missing' >&2; exit 3; }
[[ -d "$OUT_DIR/platformio" ]] || { echo 'ERROR: PlatformIO cache missing' >&2; exit 3; }
[[ -d "$OUT_DIR/esphome-cache" ]] || { echo 'ERROR: ESPHome cache missing' >&2; exit 3; }
[[ -d "$OUT_DIR/pio-components-cache" ]] || { echo 'ERROR: pio_components cache missing' >&2; exit 3; }
[[ -s "$OUT_DIR/platformio-registry-lock.json" ]] || { echo 'ERROR: PlatformIO registry lock missing' >&2; exit 3; }
[[ -d "$OUT_DIR/idf-component-mirror" ]] || { echo 'ERROR: IDF component mirror missing' >&2; exit 3; }
[[ -s "$OUT_DIR/idf-component-lock-components.txt" ]] || { echo 'ERROR: IDF component lock list missing' >&2; exit 3; }
find "$OUT_DIR/idf-component-mirror" -type f -name '*.json' -print -quit | grep -q . || { echo 'ERROR: IDF component mirror metadata missing' >&2; exit 3; }

# Avoid sharing a stale project build tree into the offline verifier. The repo
# itself remains mounted because that is exactly what will be tested.
VERIFY_RESULT="$OUT_DIR/verify-result"
rm -rf "$VERIFY_RESULT"
mkdir -p "$VERIFY_RESULT"

echo '================================================================='
echo '3/4 VERIFYING FROM SCRATCH WITH --network none'
echo '================================================================='
docker run --rm \
  --platform "$TARGET_PLATFORM" \
  --network none \
  -e "ESPHOME_VERSION=$ESPHOME_VERSION" \
  -v "$PROJECT_DIR:/work" \
  -v "$OUT_DIR:/bundle:ro" \
  -v "$VERIFY_RESULT:/verify-result" \
  -w /work \
  "$IMAGE" \
  bash /bundle/verify-inside-container.sh

[[ -s "$VERIFY_RESULT/OFFLINE_VERIFICATION_SUCCESS" ]] || {
  echo 'ERROR: network-off verification did not create OFFLINE_VERIFICATION_SUCCESS' >&2
  exit 4
}

# Copy the verification marker into the actual bundle root, then remove the
# temporary writable result directory.
cp "$VERIFY_RESULT/OFFLINE_VERIFICATION_SUCCESS" "$OUT_DIR/OFFLINE_VERIFICATION_SUCCESS"
rm -rf "$VERIFY_RESULT"

# Helper scripts/Dockerfile are useful documentation and intentionally remain
# in the bundle so the environment can be recreated later.

echo '================================================================='
echo '4/4 ARCHIVING VERIFIED BUNDLE'
echo '================================================================='

echo
printf '%-36s %s\n' 'Bundle part' 'Size'
printf '%-36s %s\n' '-----------' '----'
for p in wheelhouse platformio esphome-cache idf-component-mirror pio-components-cache esphome-build-cache; do
  if [[ -e "$OUT_DIR/$p" ]]; then
    printf '%-36s %s\n' "$p" "$(du -sh "$OUT_DIR/$p" | awk '{print $1}')"
  fi
done

echo

echo 'Required success markers:'
ls -l "$OUT_DIR/PRIME_SUCCESS" "$OUT_DIR/OFFLINE_VERIFICATION_SUCCESS"

if command -v zip >/dev/null 2>&1; then
  (
    cd "$BASE_DIR"
    zip -qr "$ARCHIVE" "$(basename "$OUT_DIR")"
  )
else
  python3 - "$OUT_DIR" "$ARCHIVE" <<'PY'
import pathlib, shutil, sys
out_dir = pathlib.Path(sys.argv[1])
archive = pathlib.Path(sys.argv[2])
base = archive.with_suffix('')
shutil.make_archive(str(base), 'zip', root_dir=out_dir.parent, base_dir=out_dir.name)
PY
fi

echo
echo '================================================================='
echo 'COMPLETE VERIFIED OFFLINE BUNDLE READY'
echo '================================================================='
ls -lh "$ARCHIVE"
echo
echo "Upload this file:"
echo "$ARCHIVE"
