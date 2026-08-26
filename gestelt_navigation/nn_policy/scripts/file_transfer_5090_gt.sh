#!/bin/bash

# === Configuration ===
SRC_PATH="/home/guest/storage/Difflying/examples/logs/vel_tracking/$1"              # Source file or directory
DEST_USER="yanrui"             # Destination username
DEST_HOST="172.26.35.50"             # Destination host (IP or domain)
#DEST_HOST="172.26.56.109" 
DEST_PATH="/home/yanrui/storage/Difflying/examples/logs/vel_tracking"             # Destination path on remote machine
FOLDER_NAME="vel_tracking"
# === Transfer Command ===
echo "Transferring '$SRC_PATH' to '$DEST_USER@$DEST_HOST:$DEST_PATH'..."
# rsync -avz yanrui@$DEST_HOST:/home/yanrui/tempstorage/perceptive_diff_drone/examples/logs/$FOLDER_NAME/$1 /home/yanrui/storage/Difflying/examples/logs/$FOLDER_NAME
rsync -avz yanrui@$DEST_HOST:/home/yanrui/storage/perceptive_diff_drone/logs/$FOLDER_NAME/$1 /home/yanrui/storage/gestelt_ws/src/gestelt/gestelt_navigation/nn_policy/logs/gate_traversal




# === Done ===
if [ $? -eq 0 ]; then
    echo "✅ Transfer complete!"
else
    echo "❌ Transfer failed."
fi
