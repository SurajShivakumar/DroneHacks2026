#!/usr/bin/env bash
# Local Ubuntu 22.04 proxy only; native CPU stereo-inertial estimator dependencies.
set -euo pipefail
source /etc/os-release
test "$VERSION_ID" = 22.04 && test "$(id -u)" = 0
export DEBIAN_FRONTEND=noninteractive
apt-get install -y --no-install-recommends libceres-dev libboost-all-dev
