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
import socket

# Overlay text onto user interface
# Input original image, text color, and text contents
# Returns new image with overlayed text
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

# Crops eyebrows from image
# Input original image and mediapipe face keypoint coordinates
# Returns 50x100 px image containing only eyebrows
def cropDetection(image_input,detection):
    # Example from from https://stackoverflow.com/questions/71094744/how-to-crop-face-detected-via-mediapipe-in-python
    image_rows, image_cols, _ = image_input.shape
    location = detection.location_data
    # Keypoint in order (right eye, left eye, nose tip, mouth center, right ear tragion, and left ear tragion) 

    # Get bounding box coordinates
    # Not used since transitioning to eyebrows only instead of full face
    """
    relative_bounding_box = location.relative_bounding_box
    rect_start_point = _normalized_to_pixel_coordinates(
        relative_bounding_box.xmin, relative_bounding_box.ymin, image_cols,
        image_rows)
    rect_end_point = _normalized_to_pixel_coordinates(
        relative_bounding_box.xmin + relative_bounding_box.width,
        relative_bounding_box.ymin + relative_bounding_box.height, image_cols,
        image_rows)
    """

    leftEar = location.relative_keypoints[5]
    leftEarPoint = _normalized_to_pixel_coordinates(
        leftEar.x, leftEar.y, image_cols,
        image_rows)

    rightEar = location.relative_keypoints[4]
    rightEarPoint = _normalized_to_pixel_coordinates(
        rightEar.x, rightEar.y, image_cols,
        image_rows)

    leftEye = location.relative_keypoints[1]
    leftEyePoint = _normalized_to_pixel_coordinates(
        leftEye.x, leftEye.y, image_cols,
        image_rows)

    rightEye = location.relative_keypoints[0]
    rightEyePoint = _normalized_to_pixel_coordinates(
        rightEye.x, rightEye.y, image_cols,
        image_rows)

    # crop image depending on distance between left and right eye
    try:

        xrightEye_relative,yrightEye_relative = rightEyePoint
        xleftEye_relative,yleftEye_relative = leftEyePoint

        xrightEar_relative,yrightEar_relative = rightEarPoint
        xleftEar_relative,yleftEar_relative = leftEarPoint

        yEyeDiff = yrightEye_relative - yleftEye_relative
        xEyeDiff = xrightEye_relative - xleftEye_relative

        xleft = xrightEye_relative + xEyeDiff/2
        xright = xleftEye_relative - xEyeDiff/2

        if yEyeDiff < 0:
            ytop = yrightEye_relative + xEyeDiff/1.5
            ybot = yleftEye_relative + xEyeDiff/8

        else:
            ytop = yleftEye_relative + xEyeDiff /1.5
            ybot = yrightEye_relative + xEyeDiff/8

        crop_img = image_input[int(ytop): int(ybot), int(xleft): int(xright)]
        #cv2.imshow('cropped',crop_img)
        #return crop_img

        resized_crop = cv2.resize(crop_img,(100,50))
        #cv2.imshow('resized_cropped',resized_crop)
        return resized_crop
        
    except:
        return -1

# predict eyebrow expression 
# Input cropped image and Trained model
# Return predicted expression
def checkExpression(img,model):
    #norm = cv2.normalize(img, 0, 1, cv2.NORM_MINMAX)
    #norm = cv2.normalize(img, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    prediction = model.predict(np.expand_dims(img,axis=0))
    expressions=['neutral','up','down','left','right']
    expression = expressions[np.argmax(prediction)]
    print(expression)
    return expression

model = load_model("./faceModel")

# Socket connection initialization
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((192.168.106.3, 8745))
msg = s.recv(64)
print(msg.decode("utf-8"))

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
                s.send("failed","utf-8") # failed expression
            else:
                expression = checkExpression(cropped_img, model) 
                mp_drawing.draw_detection(image, detection) 
                text = "Expression: " + expression 
                image = screenText(cv2.flip(image,1),"green",text)
                s.send(expression,"utf-8") # good expression
        else:
            text = "Expression: failed"
            image = screenText(cv2.flip(image,1),"black",text)
            s.send("failed","utf-8") # failed expression
         

        cv2.imshow('MediaPipe',image)

        keyPress = cv2.waitKey(5) & 0xFF 
        if keyPress == 27: # escape key
            break
        elif keyPress == 32: # SpaceBar
            print("spaceBar!")
            break

cap.release()


