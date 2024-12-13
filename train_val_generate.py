import os
import pandas as pd
import random
from collections import defaultdict

input_dir = "C:/Users/honga/Documents/UMASS AMHERST/Fall 2024/CS528/FP/DATA/csv"
output_dir = "C:/Users/honga/Documents/UMASS AMHERST/Fall 2024/CS528/FP/DATA/labels"

# Ensure output directory exists
os.makedirs(output_dir, exist_ok=True)

# a list of a pair (filename and label)
dataL = defaultdict(list)
for filename in os.listdir(input_dir):
      # Get label from the filename 
      label = filename.split('_')[0]
      # Store filename (without ".csv") and label
      if label in ["left", "right", "backward", "forward"]:
             dataL[label].append(filename.replace(".csv", ""))

train_data = []
val_data = []
for label, files in dataL.items(): 
      random.shuffle(files)
      train_data.extend([(file, label) for file in files[:21]])  
      val_data.extend([(file, label) for file in files[21:30]])   

train_dataframe = pd.DataFrame(train_data, columns=["filename", "label"])
val_dataframe = pd.DataFrame(val_data, columns=["filename", "label"])

train_dataframe.to_csv(os.path.join(output_dir, "train.csv"), index=False)
val_dataframe.to_csv(os.path.join(output_dir, "val.csv"), index=False)