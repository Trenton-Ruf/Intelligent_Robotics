#!/usr/bin/env python
import os
# I don't have an NVidia GPU :(
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' # disable annoying Tensorflow warnings
import keras
from keras.models import Sequential 
from keras.layers import Dense, Dropout, Flatten 
from keras.layers import Conv2D, MaxPooling2D 
from keras.layers import Rescaling
from keras.layers import RandomRotation

IMG_SIZE = 100

face_ds = keras.utils.image_dataset_from_directory(
    directory='./dataset',
    labels='inferred',
    label_mode='categorical', # catagorical for catagorical_crossentropy 
    batch_size=32, 
    image_size=(IMG_SIZE, IMG_SIZE),
    shuffle=True,
    seed=3,
    validation_split=0.2,
    subset="both"
)
train_ds, validation_ds = face_ds

data_augment = Sequential([
    RandomRotation(factor=(0.05), fill_mode="nearest")
    #might add more, but not image fliping because it needs to know if right or left eyebrow raised
    ])

data_rescale=Sequential([Rescaling(1./255)])

batch_size=32
input_shape=(IMG_SIZE, IMG_SIZE, 3) # IMG_SIZE x IMG_SIZE RGB

model = Sequential() 
model.add(data_augment) # augmented only during model.fit
model.add(data_rescale) # rescale data in model
model.add(Conv2D(   
     batch_size, 
     kernel_size = (3, 3),
     input_shape=input_shape,
     activation='relu'
))
model.add(Conv2D(64, (3, 3), activation = 'relu')) 
model.add(MaxPooling2D(pool_size = (2,2))) # 2x2 pooling, probably don't need 
model.add(Dropout(0.25)) 
model.add(Flatten()) 
model.add(Dense(128, activation = 'relu')) 
model.add(Dropout(0.5)) 
model.add(Dense(5, activation = 'softmax'))

model.compile(
    loss = keras.losses.categorical_crossentropy, 
    optimizer = keras.optimizers.Adam(learning_rate=0.001),
    metrics = ['accuracy']
)

callback = keras.callbacks.EarlyStopping(
    monitor="val_accuracy",
    min_delta=0,
    patience=50,
    verbose=0,
    mode="auto",
    baseline=None,
    restore_best_weights=True
)

model.fit(
    train_ds,
    validation_data = validation_ds,
    batch_size = 32,
    epochs = 5000, 
    verbose = 1,
    shuffle = True,
    callbacks=[callback]
)

model.save('./faceModel')
