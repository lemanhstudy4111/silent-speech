import os
import pandas as pd
import numpy as np
import seaborn as sns
import joblib
from sklearn.svm import SVC
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import classification_report, accuracy_score, confusion_matrix

# Directory paths (Update these paths according to your local directories)
train_labels_path = "../DATA/labels/train.csv" # Update directory
val_labels_path = "../DATA/labels/val.csv" # Update directory
train_dir = val_dir = "../DATA/csv"  # Update directory

# Load labels for train and validation
train_labels = pd.read_csv(train_labels_path)
val_labels = pd.read_csv(val_labels_path)

# Function to load dataset
def load_data(label_df, data_dir):
    # Empty lists to store features and labels
    features = []
    labels = []

    for _, row in label_df.iterrows():
        filename = os.path.join(data_dir, row['filename'] + ".csv")

        # Read file into pandas dataframe
        df = pd.read_csv(filename)
        df.replace('-', np.nan, inplace=True)
        df = df.astype(np.float32)
        df.fillna(df.mean(), inplace=True)

        # Keep only accelerometer and gyroscope signals
        data = df[['acce_x', 'acce_y', 'acce_z', 'gyro_x', 'gyro_y', 'gyro_z']].values.astype(np.float32)
        

        print(data)
        # Normalize data
        data = (data - data.min(axis=0)) / (data.max(axis=0) - data.min(axis=0))

        # Populate lists with normalized data and labels
        features.append(data.flatten()[:2004])
        labels.append(row.iloc[1])
    
    # print(features)

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
    plt.title('SVM Training Result')
    plt.xlabel('Prediction')
    plt.ylabel('Actual')
    plt.show()
    return svm_classifier

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
    plt.title('KNN Training Result')
    plt.xlabel('Prediction')
    plt.ylabel('Actual')
    plt.show()

    return knn_classifier

# Create the train and test sets
X_train, y_train = load_data(train_labels, train_dir)
X_test, y_test = load_data(val_labels, train_dir)

# Perform training and testing with SVM
svm_model = train_and_evaluate_svm(X_train, y_train, X_test, y_test)

# Perform training and testing with KNN
knn_model = train_and_evaluate_knn(X_train, y_train, X_test, y_test)

joblib.dump(svm_model, "svm_model2.pkl")
joblib.dump(knn_model, "knn_model2.pkl")