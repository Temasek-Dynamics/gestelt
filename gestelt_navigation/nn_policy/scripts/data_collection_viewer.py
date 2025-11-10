import pickle
import os
import pandas as pd


load_directory = "/home/yanrui/tempstorage5/rpg_flightning/data"
files = [f for f in os.listdir(load_directory) if os.path.isfile(os.path.join(load_directory, f))]
total_files = 43#len(files)
full_load_path = os.path.join(load_directory, "data_collected_" + str(total_files-1) + ".pkl")
full_csv_path = os.path.join(load_directory, "data_collected_" + str(total_files-1) + ".csv")
with open(full_load_path, "rb") as f:
    data_store = pickle.load(f)


df = pd.DataFrame.from_dict(data_store, orient="index")
print(df.head())
df.to_csv(full_csv_path, index=False)

