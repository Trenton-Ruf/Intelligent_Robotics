#!/usr/bin/env python
import cv2
import mediapipe as mp
from mediapipe.python.solutions.drawing_utils import _normalized_to_pixel_coordinates
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

import os
# I don't have an NVidia GPU :(
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' # disable annoying Tensorflow warnings
import keras
#from keras.models import Sequential 
from keras.models import load_model
import numpy as np

def screenText(img,color,text):
    if color.lower() == "green":
        font_color = (0,255,0)
    elif color.lower() == "black":
        font_color = (0,0,0)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_size = 0.8
    font_thickness = 2
    x,y = 0,100
    img_text = cv2.putText(img, text, (x,y), font, font_size, font_color, font_thickness, cv2.LINE_AA)
    return img_text

def cropDetection(image_input,detection):
    # Yoinked from https://stackoverflow.com/questions/71094744/how-to-crop-face-detected-via-mediapipe-in-python
    image_rows, image_cols, _ = image_input.shape
    location = detection.location_data

    relative_bounding_box = location.relative_bounding_box
    rect_start_point = _normalized_to_pixel_coordinates(
        relative_bounding_box.xmin, relative_bounding_box.ymin, image_cols,
        image_rows)
    rect_end_point = _normalized_to_pixel_coordinates(
        relative_bounding_box.xmin + relative_bounding_box.width,
        relative_bounding_box.ymin + relative_bounding_box.height, image_cols,
        image_rows)

    try:
        xleft,ytop=rect_start_point
        xright,ybot=rect_end_point

        crop_img = image_input[ytop: ybot, xleft: xright]
        resized_crop = cv2.resize(crop_img,(100,100))
        #cv2.imshow('cropped',resized_crop)
        return resized_crop
    except:
        return -1

def checkExpression(img,model):
    #norm = cv2.normalize(img, 0, 1, cv2.NORM_MINMAX)
    norm = cv2.normalize(img, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    prediction = model.predict(np.expand_dims(img,axis=0))
    expressions=['neutral','up','down','left','right']
    expression = expressions[np.argmax(prediction)]
    print(expression)
    return expression
    

model = load_model("./faceModel")

# For webcam input:
cap = cv2.VideoCapture(0)
with mp_face_detection.FaceDetection(
    model_selection=0, min_detection_confidence=0.5) as face_detection:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            continue

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = face_detection.process(image)

        # Draw the face detection annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.detections:
            detection = results.detections[0] # Grab only the closest face
            cropped_img = cropDetection(image,detection)
            if isinstance(cropped_img, int):
                text = "Expression: failed"
                image = screenText(cv2.flip(image,1),"black",text)
            else:
                expression = checkExpression(cropped_img, model) 
                mp_drawing.draw_detection(image, detection) 
                text = "Expression: " + expression 
                image = screenText(cv2.flip(image,1),"green",text)
        else:
            text = "Expression: failed"
            image = screenText(cv2.flip(image,1),"black",text)

        cv2.imshow('MediaPipe',image)

        keyPress = cv2.waitKey(5) & 0xFF 
        if keyPress == 27: # escape key
            break
        elif keyPress == 32: # SpaceBar
            print("spaceBar!")
            break

cap.release()


