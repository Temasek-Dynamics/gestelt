#!/bin/bash

# === Configuration ===
# Pushes the saved 64x64 policy depth frames (depth_debug/<run>/*.png|*.npy) to the remote box.
SRC_ROOT="/home/yanrui/storage/gestelt_ws/src/gestelt/gestelt_navigation/nn_policy/depth_debug"
DEST_USER="yanrui"
DEST_HOST="172.16.165.224"
DEST_PATH="/home/yanrui/tempstorage3/perceptive_diff_drone/depth_debug"

# Usage:
#   ./file_transfer_depth_debug.sh                  # send every run folder
#   ./file_transfer_depth_debug.sh 20260717-192714  # send just that run
if [ -n "$1" ]; then
    SRC="$SRC_ROOT/$1"
    if [ ! -d "$SRC" ]; then
        echo "❌ No such run folder: $SRC"
        echo "Available:"; ls -1 "$SRC_ROOT"
        exit 1
    fi
else
    SRC="$SRC_ROOT/"
fi

echo "Transferring '$SRC' to '$DEST_USER@$DEST_HOST:$DEST_PATH'..."
# --rsync-path creates the destination on the remote if it doesn't exist yet.
rsync -avz --progress \
      --rsync-path="mkdir -p $DEST_PATH && rsync" \
      "$SRC" "$DEST_USER@$DEST_HOST:$DEST_PATH/"

# === Done ===
if [ $? -eq 0 ]; then
    echo "✅ Transfer complete!"
else
    echo "❌ Transfer failed."
fi
