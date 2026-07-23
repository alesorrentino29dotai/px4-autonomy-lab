#!/usr/bin/env bash
# Fetch the prebuilt MAVSDK C++ library into cpp/third_party/mavsdk (no sudo:
# the .deb is extracted with dpkg -x, not installed system-wide).
set -euo pipefail
VERSION="${VERSION:-3.17.2}"
FLAVOR="${FLAVOR:-ubuntu24.04}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEB="/tmp/libmavsdk-dev_${VERSION}_${FLAVOR}_amd64.deb"
URL="https://github.com/mavlink/MAVSDK/releases/download/v${VERSION}/libmavsdk-dev_${VERSION}_${FLAVOR}_amd64.deb"

echo "fetching ${URL}"
curl -fsSL -o "$DEB" "$URL"
rm -rf "${DIR}/third_party/mavsdk"
mkdir -p "${DIR}/third_party/mavsdk"
dpkg -x "$DEB" "${DIR}/third_party/mavsdk"
# ldconfig would normally create these on a real install
( cd "${DIR}/third_party/mavsdk/usr/lib" &&
  ln -sf "libmavsdk.so.${VERSION}" libmavsdk.so.3 &&
  ln -sf libmavsdk.so.3 libmavsdk.so )
echo "OK: $(ls "${DIR}"/third_party/mavsdk/usr/lib/libmavsdk.so.*)"
