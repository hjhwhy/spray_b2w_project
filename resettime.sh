#!/bin/bash

set -euo pipefail

usage() {
    echo "Usage: $0 -M-D [HH:MM:SS]"
    echo "Example: $0 -4-29"
    echo "Example: $0 -4-29 08:30:00"
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
    usage >&2
    exit 1
fi

date_arg="$1"
time_arg="${2:-00:00:00}"

if [[ ! "$date_arg" =~ ^-([0-9]{1,2})-([0-9]{1,2})$ ]]; then
    usage >&2
    exit 1
fi

month="${BASH_REMATCH[1]}"
day="${BASH_REMATCH[2]}"

if (( month < 1 || month > 12 || day < 1 || day > 31 )); then
    echo "Invalid date: month=$month day=$day" >&2
    exit 1
fi

if [[ ! "$time_arg" =~ ^[0-9]{1,2}:[0-9]{2}:[0-9]{2}$ ]]; then
    echo "Invalid time: $time_arg" >&2
    usage >&2
    exit 1
fi

target_time=$(printf '2026-%02d-%02d %s' "$month" "$day" "$time_arg")

echo "Disable NTP..."
sudo timedatectl set-ntp false

echo "Set system time: $target_time"
sudo timedatectl set-time "$target_time"

timedatectl status
