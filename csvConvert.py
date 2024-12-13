import os
import pandas as pd

# Function to convert from txt to csv
input_dir = "C:/Users/honga/Documents/UMASS AMHERST/Fall 2024/CS528/Assignment 2/DATA/txt"
output_dir = "C:/Users/honga/Documents/UMASS AMHERST/Fall 2024/CS528/Assignment 2/DATA/csv"

# Ensure output directory exists
os.makedirs(output_dir, exist_ok=True)

for folder_name in os.listdir(input_dir):
    folder_path = os.path.join(input_dir, folder_name)  
    # Iterate through each file in the folder
    for filename in os.listdir(folder_path):
        txt_file_path = os.path.join(folder_path, filename)
        df = pd.read_csv(txt_file_path, sep=r'\s+')
        csv_file_path = os.path.join(output_dir, filename.replace(".txt", ".csv"))
        df.to_csv(csv_file_path, index=False)
        print(f"Converted {txt_file_path} to {csv_file_path}")


import os
import numpy as np
import pandas as pd
import seaborn as sns
from sklearn.svm import SVC
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import classification_report, accuracy_score, confusion_matrix
from google.colab import drive
drive.mount('/content/drive')
train_labels = pd.read_csv("/content/drive/My Drive/Fall 2024/COMPSCI 528/Assignment 2/Data/labels/train.csv") # Update directory
val_labels = pd.read_csv("/content/drive/My Drive/Fall 2024/COMPSCI 528/Assignment 2/Data/labels/val.csv") # Update directory
train_dir = val_dir = "/content/drive/My Drive/Fall 2024/COMPSCI 528/Assignment 2/Data/csv" # Update directory
# Function to load dataset
def load_data(label_df, data_dir):
    # Empty lists to store features and labels
    features = []
    labels = []

    for _, row in label_df.iterrows():
        filename = os.path.join(data_dir, row['filename'] + ".csv")

        # Read file into pandas dataframe
        df = pd.read_csv(filename)

        # Keep only accelerometer and gyroscope signals
        data = df[['acce_x', 'acce_y', 'acce_z', 'gyro_x', 'gyro_y', 'gyro_z']].values.astype(np.float32)

        # Normalize data
        data = (data - data.min(axis=0)) / (data.max(axis=0) - data.min(axis=0))

        # Populate lists with normalized data and labels
        features.append(data.flatten())
        labels.append(row.iloc[1])

    return np.array(features), np.array(labels)

def train_and_evaluate_svm(X_train, y_train, X_test, y_test):
    # Create the SVM classifier
    svm_classifier = SVC(kernel='rbf')

    # Train the classifier
    svm_classifier.fit(X_train, y_train)

    # Perform prediction on the test set
    y_pred = svm_classifier.predict(X_test)

    # Evaluate the model
    accuracy = accuracy_score(y_test, y_pred)
    print(f'SVM accuracy: {accuracy:.3%}')

    # Plot the confusion matrix
    conf_matrix = confusion_matrix(y_test, y_pred)
    sns.heatmap(conf_matrix, annot=True, cmap="Blues")
    plt.title('Training Result')
    plt.xlabel('Prediction')
    plt.ylabel('Actual')
    plt.show()

    def train_and_evaluate_knn(X_train, y_train, X_test, y_test, n_neighbors=3):
    # Create the KNN classifier
    knn_classifier = KNeighborsClassifier(n_neighbors=n_neighbors)

    # Train the classifier
    knn_classifier.fit(X_train, y_train)

    # Perform prediction on the test set
    y_pred = knn_classifier.predict(X_test)

    # Evaluate the model
    accuracy = accuracy_score(y_test, y_pred)
    print(f'KNN accuracy: {accuracy:.3%}')

    # Plot the confusion matrix
    conf_matrix = confusion_matrix(y_test, y_pred)
    sns.heatmap(conf_matrix, annot=True, cmap="Blues")
    plt.title('Training Result')
    plt.xlabel('Prediction')
    plt.ylabel('Actual')
    plt.show()

    # Create the train and test sets
X_train, y_train = load_data(train_labels, train_dir)
X_test, y_test = load_data(val_labels, train_dir)

# Perform training and testing with SVM
train_and_evaluate_svm(X_train, y_train, X_test, y_test)

# Perform training and testing with KNN
train_and_evaluate_knn(X_train, y_train, X_test, y_test)