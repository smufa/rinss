import cv2
import face_recognition
import numpy as np

def deskew_image(img, angle):
    h, w = image.shape[:2]

    m = cv2.getRotationMatrix2D((w/2, h/2), angle, 1)

    return cv2.warpAffine(img, m, (w,h))

# Load the pre-trained face detection classifier
face_cascade = cv2.CascadeClassifier('/home/nana/ROS/srcLanTask1/exercise4/scripts/haarcascade_frontalface_default.xml')

# Load the image
image = cv2.imread('/home/nana/ROS/src/task2/meshes/neki4.png')

cv2.imshow('Face Detection', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

deskew = deskew_image(image, 20)
cv2.imshow('Face Detection', deskew)
cv2.waitKey(0)
cv2.destroyAllWindows()


# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Perform face detection
faces = face_cascade.detectMultiScale(image, scaleFactor=1.01, minNeighbors=10)

# Draw rectangles around the detected faces
offset = 10
for (x, y, w, h) in faces:
    # cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 3)
    
    adjusted = image[y-offset:y+h+offset,x-offset:x+w+offset]

    alpha = 3  # contrast:  [0,1) => lower    [1,] => higher
    beta = 2   # brightness
    adjusted = cv2.convertScaleAbs(adjusted, alpha, beta)

    # scale_factor = 2
    # adjusted = cv2.resize(cropped, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_CUBIC)
    cv2.imshow('Face Detection', adjusted)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # loc = face_recognition.face_locations(adjusted, model='cnn')
    camera_encoding = face_recognition.face_encodings(image, model='large')
    if len(camera_encoding) != 0:
        camera_encoding = camera_encoding[0]
        cv2.imshow('Face Detection', image[y:y+h,x:x+w])
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print('nism nausu sm faco')

    # Display the image with detected faces
    # cv2.imshow('Face Detection', camera_encoding)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

