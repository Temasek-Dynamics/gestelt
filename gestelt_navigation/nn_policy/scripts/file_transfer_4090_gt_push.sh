#!/bin/bash

# === Configuration ===
# Pushes a gate_traversal policy run (local logs/gate_traversal/<run>) to 172.16.165.224,
# to the same remote path that file_transfer_4090_gt.sh pulls FROM (so a later pull
# with that script finds it in the right place).
SRC_ROOT="/home/yanrui/storage/gestelt_ws/src/gestelt/gestelt_navigation/nn_policy/logs/gate_traversal"
DEST_USER="yanrui"
DEST_HOST="172.16.165.224"
FOLDER_NAME="vel_tracking"
DEST_PATH="/home/yanrui/tempstorage/perceptive_diff_drone/logs/$FOLDER_NAME"

# Usage: ./file_transfer_4090_gt_push.sh 20260705-155924
if [ -z "$1" ]; then
    echo "Usage: $0 <run_name>   e.g. $0 20260705-155924"
    echo "Available local runs:"; ls -1 "$SRC_ROOT"
    exit 1
fi

SRC="$SRC_ROOT/$1"
if [ ! -d "$SRC" ]; then
    echo "❌ No such local run folder: $SRC"
    exit 1
fi

echo "Transferring '$SRC' to '$DEST_USER@$DEST_HOST:$DEST_PATH/$1'..."
rsync -avz --progress \
      --rsync-path="mkdir -p $DEST_PATH && rsync" \
      "$SRC" "$DEST_USER@$DEST_HOST:$DEST_PATH/"

# === Done ===
if [ $? -eq 0 ]; then
    echo "✅ Transfer complete!"
else
    echo "❌ Transfer failed."
fi
