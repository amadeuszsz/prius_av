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
import io
from pathlib import Path
from datetime import datetime
import yaml
import cv2
import numpy as np
import pandas as pd
import tensorflow as tf
import matplotlib.pyplot as plt
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.preprocessing.image import ImageDataGenerator

physical_devices = tf.config.list_physical_devices("GPU")
tf.config.experimental.set_memory_growth(physical_devices[0], True)
stream = open(os.path.join(os.path.abspath('..'), "params/prius.yaml"), "r")
dataset_path = os.path.join(os.path.abspath('..'), "data/front_camera")
training_config = yaml.load(stream, Loader=yaml.FullLoader)
model_dir = os.path.join(Path(__file__).parent.parent, "models", "steering")
log_dir = os.path.join(model_dir, "logs/fit/", datetime.now().strftime("%Y%m%d-%H%M%S"))
checkpoint_dir = os.path.join(model_dir, "checkpoint")
file_writer = tf.summary.create_file_writer(log_dir)
batch_size = 32
epochs = 10
validation_split = 0.3

df = pd.read_csv(os.path.join(dataset_path, "data.csv"), dtype={"no.": "string"})
df["no."] = df["no."].apply(lambda x: x + ".jpg")
df["velocity"] = df["velocity"].apply(lambda x: x / (training_config["velocity_ms_max"] / 2) - 1)  # normalize velocity (-1, 1)
df_size = len(df)
compute_steps_per_epoch = lambda x: int(math.ceil(1. * x / batch_size))
steps_size_train = compute_steps_per_epoch(df_size * (1 - validation_split))
steps_size_valid = compute_steps_per_epoch(df_size * validation_split)


def get_nvidia_model():
    inputs = keras.Input(shape=(66, 200, 3))
    # 5x5 Convolutional layers with stride of 2x2
    x = layers.Conv2D(filters=24, kernel_size=(5, 5), strides=(2, 2), activation='relu')(inputs)
    x = layers.Conv2D(filters=36, kernel_size=(5, 5), strides=(2, 2), activation='relu')(x)
    x = layers.Conv2D(filters=48, kernel_size=(5, 5), strides=(2, 2), activation='relu')(x)
    # 3x3 Convolutional layers with stride of 1x1
    x = layers.Conv2D(filters=64, kernel_size=(3, 3), strides=(1, 1), activation='relu')(x)
    x = layers.Conv2D(filters=64, kernel_size=(3, 3), strides=(1, 1), activation='relu')(x)
    # Flatten before passing to the fully connected layers
    x = layers.Dropout(rate=0.5)(x)
    x = layers.Flatten()(x)
    # Three fully connected layers
    x = layers.Dense(units=100, activation='relu')(x)
    x = layers.Dense(units=50, activation='relu')(x)
    x = layers.Dense(units=10, activation='relu')(x)
    # Output layer with tanh activation
    outputs = layers.Dense(units=1, activation='tanh')(x)
    nvidia_model = keras.Model(inputs=inputs, outputs=outputs)
    return nvidia_model


def preprocessing_function(img):
    # Gaussian blur
    img = cv2.GaussianBlur(img, (3, 3), 0)
    # Change to YUV image
    img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    # Decrease size for easier processing
    #img = cv2.resize(img, (66, 200))
    return img


# def prediction_preview(logs):
#     with file_writer.as_default():
#         tb_images = np.array(
#             [cv2.cvtColor(sample * 255, cv2.COLOR_YUV2RGB) for sample in valid_generator[0][0]]).astype(np.uint8)
#         tb_predictions = model(valid_generator[0][0])
#         tf.summary.image("Validation data examples", tb_images, max_outputs=32, step=1)


def plot_to_image(figure):
    """Converts the matplotlib plot specified by 'figure' to a PNG image and
    returns it. The supplied figure is closed and inaccessible after this call."""
    # Save the plot to a PNG in memory.
    buf = io.BytesIO()
    plt.savefig(buf, format='png')
    # Closing the figure prevents it from being displayed directly inside
    # the notebook.
    plt.close(figure)
    buf.seek(0)
    # Convert PNG buffer to TF image
    image = tf.image.decode_png(buf.getvalue(), channels=4)
    # Add the batch dimension
    image = tf.expand_dims(image, 0)
    return image


def image_grid():
    """Return a 5x5 grid of the MNIST images as a matplotlib figure."""
    tb_images = np.array([cv2.cvtColor(sample * 255, cv2.COLOR_YUV2BGR) for sample in valid_generator[0][0]]).astype(np.uint8)
    tb_labels = valid_generator[0][1]
    tb_predictions = model(valid_generator[0][0])

    # Create a figure to contain the plot.
    figure = plt.figure(figsize=(20,20))
    for key, value in enumerate(tb_images):
        # Start next subplot.
        plt.subplot(8, 4, key + 1, title="Lab: {:.2f}, Pred: {:.2f}".format(tb_labels[key, 0], tb_predictions[key, 0]))
        plt.xticks([])
        plt.yticks([])
        plt.grid(False)
        plt.imshow(tb_images[key])

    return figure


def get_single_predict(image_name="0000000001.jpg"):
    # Load image
    img = cv2.imread(os.path.join(dataset_path, image_name))
    # Change to YUV image
    img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    # Gaussian blur
    img = cv2.GaussianBlur(img, (3, 3), 0)
    # Decrease size for easier processing
    img = cv2.resize(img, (200, 66))
    # Normalize values
    img = (img / 255)

    prediction = model(tf.cast(img[tf.newaxis, ...], dtype=np.float32))
    steer_label = df["steer"][int(image_name.split(".")[0])]
    velocity_label = df["velocity"][int(image_name.split(".")[0])]
    steer_prediction = prediction[0, 0]
    velocity_prediction = prediction[0, 1]
    print("Steer l/p: {}/{}\nVelocity l/p: {}/{}\n".format(steer_label, steer_prediction, velocity_label, velocity_prediction))


datagen = ImageDataGenerator(
    rotation_range=5,
    width_shift_range=0.05,
    height_shift_range=0.05,
    zoom_range=(0.9, 0.9),
    rescale=1.0 / 255,
    preprocessing_function=preprocessing_function,
    data_format="channels_last",
    validation_split=validation_split,
    dtype=np.float32,
)


train_generator = datagen.flow_from_dataframe(
    dataframe=df,
    directory=dataset_path,
    x_col="no.",
    #y_col=["steer", "velocity"],
    y_col=["steer"],
    target_size=(66, 200),
    class_mode="raw",
    batch_size=batch_size,
    shuffle=True,
    seed=100,
    save_format="jpg",
    subset="training"
)


valid_generator = datagen.flow_from_dataframe(
    dataframe=df,
    directory=dataset_path,
    x_col="no.",
    #y_col=["steer", "velocity"],
    y_col=["steer"],
    target_size=(66, 200),
    class_mode="raw",
    batch_size=batch_size,
    shuffle=True,
    seed=100,
    save_format="jpg",
    subset="validation"
)

tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1)

model_checkpoint_callback = tf.keras.callbacks.ModelCheckpoint(
    filepath=checkpoint_dir,
    monitor='val_loss',
    mode='min',
    save_best_only=True
)

model = get_nvidia_model()

model.compile(
    optimizer=keras.optimizers.Adam(),
    loss=tf.keras.losses.MeanSquaredError(),
)

model.fit(
    train_generator,
    epochs=epochs,
    steps_per_epoch=steps_size_train,
    validation_data=valid_generator,
    validation_steps=steps_size_valid,
    callbacks=[tensorboard_callback, model_checkpoint_callback]
)

# Prepare the plot
figure = image_grid()
# Convert to image and log
with file_writer.as_default():
  tf.summary.image("Validation data", plot_to_image(figure), step=0)
