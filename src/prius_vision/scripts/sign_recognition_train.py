#!/usr/bin/env python3
"""
Copyright 2020 Amadeusz Szymko <amadeusz.szymko@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
"""

import os
import math
from pathlib import Path
from datetime import datetime
import cv2
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.utils import to_categorical
from sklearn.model_selection import train_test_split

# Network params
batch_size = 32
epochs = 50
validation_split = 0.3
input_shape = (50, 50)

# Paths
physical_devices = tf.config.list_physical_devices("GPU")
tf.config.experimental.set_memory_growth(physical_devices[0], True)
dataset_path = os.path.join(os.path.abspath('..'), "data/signs")
model_dir = os.path.join(Path(__file__).parent.parent, "models", "sign_recognition")
log_dir = os.path.join(model_dir, "logs/fit/", datetime.now().strftime("%Y%m%d-%H%M%S"))
checkpoint_dir = os.path.join(model_dir, "checkpoint")
file_writer = tf.summary.create_file_writer(log_dir)
root = os.path.join(os.path.expanduser("~"), "prius_av/src/prius_vision/data/signs")

# Get imgs & labels
img_paths = [Path(os.path.join(path, name)) for path, subdirs, files in os.walk(root) for name in files if name != "data.csv"]
img_str_labels = np.array([img_path.parts[-2].split("_")[1] for img_path in img_paths])
classes = np.unique(img_str_labels).astype(int)
classes.sort()
classes = classes.astype(str)
img_labels = to_categorical(np.argmax(img_str_labels[..., np.newaxis] == classes, axis=1))

# Dataset params
compute_steps_per_epoch = lambda x: int(math.ceil(1. * x / batch_size))
steps_size_train = compute_steps_per_epoch(len(img_paths) * (1 - validation_split))
steps_size_valid = compute_steps_per_epoch(len(img_paths) * validation_split)

# Process images
imgs = [cv2.imread(img_path.as_posix()) for img_path in img_paths]
imgs = [cv2.GaussianBlur(img, (3, 3), 0) for img in imgs]
imgs = [cv2.cvtColor(img, cv2.COLOR_RGB2YUV) for img in imgs]
imgs = np.array([cv2.resize(img, input_shape) for img in imgs])/255

# Split dataset
x_train, x_validation, y_train, y_validation = train_test_split(imgs, img_labels, test_size=validation_split, random_state=100)


def get_model():
    inputs = keras.Input(shape=(input_shape[0], input_shape[1], 3))
    # 5x5 Convolutional layers
    x = layers.Conv2D(filters=32, kernel_size=(5, 5), activation='relu')(inputs)
    x = layers.Conv2D(filters=32, kernel_size=(5, 5), activation='relu')(x)
    x = layers.MaxPool2D(pool_size=(2, 2))(x)
    x = layers.Dropout(rate=0.25)(x)
    # 3x3 Convolutional layers
    x = layers.Conv2D(filters=64, kernel_size=(3, 3), activation='relu')(x)
    x = layers.Conv2D(filters=64, kernel_size=(3, 3), activation='relu')(x)
    x = layers.MaxPool2D(pool_size=(2, 2))(x)
    x = layers.Dropout(rate=0.25)(x)
    # Flatten before passing to the fully connected layers
    x = layers.Flatten()(x)
    # One fully connected layers
    x = layers.Dense(units=256, activation='relu')(x)
    x = layers.Dropout(rate=0.5)(x)
    # Output layer with softmax activation
    outputs = layers.Dense(units=8, activation='softmax')(x)
    cnn_model = keras.Model(inputs=inputs, outputs=outputs)
    return cnn_model


tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1)

model_checkpoint_callback = tf.keras.callbacks.ModelCheckpoint(
    filepath=checkpoint_dir,
    monitor='val_loss',
    mode='min',
    save_best_only=True
)

model = get_model()

model.compile(
    optimizer=keras.optimizers.Adam(),
    loss=tf.keras.losses.CategoricalCrossentropy(),
    metrics=['accuracy']
)

model.fit(
    x_train,
    y_train,
    epochs=epochs,
    batch_size=batch_size,
    steps_per_epoch=steps_size_train,
    validation_data=(x_validation, y_validation),
    validation_steps=steps_size_valid,
    callbacks=[tensorboard_callback, model_checkpoint_callback]
)
