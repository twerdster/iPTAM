#!/bin/bash
set -euo pipefail
iptam_root="$(cd "$(dirname "$0")/.." && pwd)"
iptam_variant="${1:-current}"
case "$iptam_variant" in
  current) iptam_build="$iptam_root/build-ios"; iptam_target=iPTAM ;;
  upstream) iptam_build="$iptam_root/build-ios-upstream"; iptam_target=PTAMUpstream ;;
  mobile) iptam_build="$iptam_root/build-ios-mobile"; iptam_target=PTAMMobile ;;
  *) echo "Usage: $0 [current|upstream|mobile]" >&2; exit 2 ;;
esac
iptam_upstream_port="${IPTAM_UPSTREAM_PORT_DIR:-$iptam_root/../PTAM-upstream-port}"
iptam_signing_team="${IPTAM_DEVELOPMENT_TEAM:-}"
iptam_bundle_prefix="${IPTAM_BUNDLE_PREFIX:-org.example.iptam}"
iptam_unsigned="${IPTAM_UNSIGNED:-0}"
if [ "$iptam_unsigned" != 1 ] && [ -z "$iptam_signing_team" ]; then
  echo "Set IPTAM_DEVELOPMENT_TEAM to your Apple signing team, or IPTAM_UNSIGNED=1 for an unsigned build." >&2
  exit 2
fi
cmake -S "$iptam_root/ios" -B "$iptam_build" -G Xcode \
  -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_SYSROOT=iphoneos \
  -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_DEPLOYMENT_TARGET=16.0 \
  -DIPTAM_ENGINE_VARIANT="$iptam_variant" -DIPTAM_UPSTREAM_PORT_DIR="$iptam_upstream_port" \
  -DIPTAM_BUNDLE_PREFIX="$iptam_bundle_prefix" -DIPTAM_DEVELOPMENT_TEAM="$iptam_signing_team"
python3 "$iptam_root/scripts/snapshot_source.py" --output "$iptam_build/provenance"
if [ "$iptam_variant" = upstream ]; then
  python3 "$iptam_root/scripts/snapshot_upstream.py" --port "$iptam_upstream_port" --output "$iptam_build/provenance"
fi
# This process-local setting avoids Xcode16.1's unrelated simulator runtime
# availability gate for a direct-target physical-device build.
iptam_signing_args=()
if [ "$iptam_unsigned" = 1 ]; then
  iptam_signing_args+=(CODE_SIGNING_ALLOWED=NO CODE_SIGNING_REQUIRED=NO)
else
  iptam_signing_args+=(-allowProvisioningUpdates -allowProvisioningDeviceRegistration)
fi
env RC_NO_USE_SIM_RUNTIME=1 xcodebuild -project "$iptam_build/iPTAMiOS.xcodeproj" \
  -target "$iptam_target" -configuration Release -sdk iphoneos \
  SDK_STAT_CACHE_ENABLE=NO \
  "${iptam_signing_args[@]}" build
python3 "$iptam_root/scripts/snapshot_source.py" --output "$iptam_build/provenance" --verify
if [ "$iptam_variant" = upstream ]; then
  python3 "$iptam_root/scripts/snapshot_upstream.py" --port "$iptam_upstream_port" --output "$iptam_build/provenance" --verify
fi
