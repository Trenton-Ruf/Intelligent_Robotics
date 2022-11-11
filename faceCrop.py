#!/usr/bin/env python
import cv2
from pathlib import Path
import mediapipe as mp
from mediapipe.python.solutions.drawing_utils import _normalized_to_pixel_coordinates
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

# https://google.github.io/mediapipe/solutions/face_detection

expressions=['neutral','up','down','left','right']
expression_count = 0
capturing = False

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
    # Keypoint in order (right eye, left eye, nose tip, mouth center, right ear tragion, and left ear tragion) 

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

def saveExpression(img,expression,count):
    path = ("./dataset/" + expression)
    filename = path + "/" + str(count) + ".jpg"

    # Create Path
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)

    # Save Image
    if not cv2.imwrite(filename, img) :
        print("image did not save")

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

        if capturing is False: 
            captureCount = 0
            text = "Press Space to capture " + expressions[expression_count] + " expression." 
            image = screenText(cv2.flip(image,1),"black",text)
        else:
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
                saveExpression(cropped_img, expressions[expression_count - 1], captureCount)
                mp_drawing.draw_detection(image, detection) 
                captureCount += 1
            text = "Capturing " + expressions[expression_count - 1] + " " +  str(captureCount) + "." 
            image = screenText(cv2.flip(image,1),"green",text)
            image[0:50,0:100,:] = cropped_img
            if captureCount >= 500:
                capturing = False
                if expression_count > 4:
                    break

        cv2.imshow('MediaPipe',image)

        keyPress = cv2.waitKey(5) & 0xFF 
        if keyPress == 27: # escape key
            break
        elif keyPress == 32: # SpaceBar
            print("spaceBar!")
            if capturing is False:
                expression_count += 1 
                capturing = True

cap.release()

#cv2.resize(image,new_dimension)
