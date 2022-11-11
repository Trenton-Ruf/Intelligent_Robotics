#!/usr/bin/env python
import os
# I don't have an NVidia GPU :(
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' # disable annoying Tensorflow warnings
import tensorflow as tf
import keras
from keras.models import Sequential 
from keras.layers import Dense, Dropout, Flatten 
from keras.layers import Conv2D, MaxPooling2D 
from keras.layers import Rescaling
from keras.layers import RandomRotation, RandomZoom, RandomBrightness, RandomContrast, RandomCrop

from matplotlib import pyplot as plt
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
import numpy as np

expressions=['neutral','up','down','left','right']

face_ds = keras.utils.image_dataset_from_directory(
    directory='./dataset',
    labels='inferred',
    class_names=expressions,
    label_mode='categorical', # catagorical for catagorical_crossentropy 
    batch_size=32, 
    image_size=(50, 100),
    shuffle=True,
    seed=3,
    validation_split=0.1,
    subset="both"
)
train_ds, validation_ds = face_ds

data_augment = Sequential([
    RandomRotation(factor=(0.05), fill_mode="nearest"),
    RandomZoom(height_factor=(0.05),width_factor=(0.05), fill_mode="nearest"),
    RandomContrast(factor=(0.05)),
    RandomBrightness(factor=(0.05))
    #might add more, but not image fliping because it needs to know if right or left eyebrow raised
    ])

data_rescale=Sequential([Rescaling(1./255)])

input_shape=(50, 100, 3) # IMG_SIZE x IMG_SIZE RGB

model = Sequential() 
model.add(data_augment) # augmented only during model.fit
model.add(data_rescale) # rescale data in model
model.add(Conv2D(   
     32, 
     kernel_size = (3, 3),
     input_shape=input_shape,
     activation='relu'
))
model.add(Conv2D(64, (3, 3), activation = 'relu')) 
model.add(MaxPooling2D(pool_size = (2,2))) # 2x2 pooling, probably don't need 
model.add(Dropout(0.25)) 
model.add(Flatten()) 
model.add(Dense(256, activation = 'relu')) 
model.add(Dropout(0.5)) 
model.add(Dense(256, activation = 'relu')) 
model.add(Dropout(0.5)) 
model.add(Dense(5, activation = 'softmax'))

model.compile(
    loss = keras.losses.categorical_crossentropy, 
    #optimizer = keras.optimizers.Adam(learning_rate=0.001),
    optimizer = keras.optimizers.Adadelta(learning_rate=0.001),
    metrics = ['accuracy']
)

callback = keras.callbacks.EarlyStopping(
    monitor="val_loss",
    min_delta=0,
    patience=15,
    verbose=0,
    mode="auto",
    baseline=None,
    restore_best_weights=True
)

history = model.fit(
    train_ds,
    validation_data = validation_ds,
    batch_size = 32,
    epochs = 5000, 
    verbose = 1,
    shuffle = True,
    callbacks=[callback]
)

#print(history.history.keys())

#  Need to save training graph
plt.plot(history.history['accuracy'], label='Training Accuracy')
plt.plot(history.history['val_accuracy'], label='Validation Accuracy')
plt.plot(history.history['loss'], label='Training Loss')
plt.plot(history.history['val_loss'], label='validation Loss')
plt.title('Eyebrow Training History')
plt.xlabel('Epoch')
#plt.ylabel('Y')
plt.legend(loc='best')  # legend text comes from the plot's label parameter.
plt.savefig("./trainingGraph.png",transparent=True)

#  Might as well put the Confusion Matrix too.
y_test = np.concatenate([y for x, y in validation_ds], axis=0)
pred = model.predict(validation_ds)

plt.clf()
cm = confusion_matrix(np.argmax(y_test, axis=-1), np.argmax(pred, axis=-1) )
disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=expressions)
disp.plot()
plt.savefig("./confusionMatrix.png", transparent=True)

model.save('./faceModel')
