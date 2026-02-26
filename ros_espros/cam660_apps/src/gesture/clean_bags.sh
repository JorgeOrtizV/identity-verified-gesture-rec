#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <path>"
    exit 1
fi

PATH_DIR="$1"
count=1

if [ ! -d "$PATH_DIR" ]; then
    echo "Error: '$PATH_DIR' is not a valid directory"
    exit 1
fi

BAG_FILES=("$PATH_DIR"/*.bag)

if [ ! -e "${BAG_FILES[0]}" ]; then
    echo "No .bag files found in '$PATH_DIR'"
    exit 0
fi

for BAG in "${BAG_FILES[@]}"; do
    if (( $count == 42 )); then
        echo "$(basename "$BAG")"
	exit 0
    fi
    count=$((count + 1))
    #echo "=== $(basename "$BAG") ==="
    #rosbag info "$BAG" -y -k duration
    #echo ""
done
