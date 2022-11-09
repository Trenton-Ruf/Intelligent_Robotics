#!/usr/bin/env python
import os
# I don't have an NVidia GPU :(
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' # disable annoying Tensorflow warnings
import tensorflow as tf
import keras
from keras.models import Sequential 
from keras.layers import Dense, Dropout, Flatten 
from keras.layers import Conv2D, MaxPooling2D 

def pre_process(image,label):
    image = tf.cast(image/255. ,tf.float32)
    return image,label

face_ds = keras.utils.image_dataset_from_directory(
    directory='./dataset',
    labels='inferred',
    label_mode='categorical', # catagorical for catagorical_crossentropy 
    #label_mode='int', #for sparse_catagorical_crossentropy
    batch_size=32, # maybe None
    image_size=(100, 100),
    shuffle=True,
    seed=3,
    validation_split=0.2,
    subset="both"
)
train_ds, validation_ds = face_ds

train_ds = train_ds.map(pre_process)
validation_ds = validation_ds.map(pre_process)

batch_size=32
input_shape=(100, 100, 3) # 100x100 RGB

model = Sequential() 
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
