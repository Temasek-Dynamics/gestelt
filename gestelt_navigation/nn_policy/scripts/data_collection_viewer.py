import pickle
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


load_directory = "/home/yanrui/tempstorage5/rpg_flightning/data"
files = [f for f in os.listdir(load_directory) if os.path.isfile(os.path.join(load_directory, f))]
total_files = 8#len(files)
full_load_path = os.path.join(load_directory, "data_collected_" + str(total_files-1) + ".pkl")
full_csv_path = os.path.join(load_directory, "data_collected_" + str(total_files-1) + ".csv")
with open(full_load_path, "rb") as f:
    data_store = pickle.load(f)


plt.figure()
for curr_data in data_store:
    curr_position_list = []
    counter = 0
    for ii in range(len(curr_data)):
        curr_data_position = curr_data[counter]["position"]
        curr_position_list.append(curr_data_position)
        counter += 0.02
    
    curr_position_array = np.array(curr_position_list[1:-1])
    xs = curr_position_array[:, 0]
    ys = curr_position_array[:, 1]
    plt.plot(xs, ys)

plt.xlabel("X")
plt.ylabel("Y")
plt.title("2D Trajectories")
plt.grid(True)
plt.show()


# df = pd.DataFrame.from_dict(data_store, orient="index")
# print(df.head())
# df.to_csv(full_csv_path, index=False)

