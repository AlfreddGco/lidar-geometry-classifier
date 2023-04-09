import numpy as np
import tensorflow as tf
from tensorflow import keras
from keras import layers

RESOURCE = 'src/lidar_planning/resource'
CLUSTERS_PATH = RESOURCE + '/clusters'

# Define the model
model = keras.Sequential([
    keras.layers.Input(shape=(256, 3)),
    keras.layers.Conv1D(1, 20, input_shape=(256, 3), activation='tanh'),
    # shape (256 - 20 + 1, 1) = (237, 1)
    keras.layers.Conv1D(1, 20, input_shape=(237, 1), activation='tanh'),
    # shape (237 - 20 + 1, 1) = (218, 1)
    keras.layers.Flatten(),
    keras.layers.Dense(218, activation='relu'),
    keras.layers.Dense(128, activation='relu'),
    keras.layers.Dense(64, activation='relu'),
    keras.layers.Dense(1, activation='sigmoid')
])

if __name__ == '__main__':
    # Compile the model
    model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])
    model.summary()

    x_cube = []
    with open(CLUSTERS_PATH + '/cubes.npy', 'rb') as f:
        x_cube = np.load(f)
    y_cube = np.full(len(x_cube), 1)

    x_other = []
    with open(CLUSTERS_PATH + '/other_shapes.npy', 'rb') as f:
        x_other = np.load(f)
    y_other = np.full(len(x_other), 0)

    x = np.concatenate((x_cube, x_other))
    y = np.concatenate((y_cube, y_other))

    perm = np.random.permutation(len(x))
    x_train, y_train = x[perm], y[perm]
    print('Training data:', len(x_train))
    model.fit(x_train, y_train, epochs=40, batch_size=32, validation_split=0.2)
    model.save_weights('cube_detector_weights.h5')

def load_model():
    model.load_weights('./cube_detector_weights.h5')
    return lambda points: model.predict(np.array([points]))[0][0]


def generate_cluster_data(clusters, filename, k = 1):
    sample = []
    for cluster in clusters:
        for i in range(512*k):
            sample.append(cluster.random_walk())
    with open(filename, 'wb') as f:
        np.save(f, np.array(sample))

