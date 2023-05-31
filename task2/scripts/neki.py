import cv2
import numpy as np
# from keras_facenet import FaceNet
print('tu')
def compare_faces(img1_path, img2_path):
    # Load the images
    img = cv2.imread(img1_path, cv2.IMREAD_GRAYSCALE)
    wantedImg = cv2.imread(img2_path, cv2.IMREAD_GRAYSCALE)

    face_cascade = cv2.CascadeClassifier('/home/nana/ROS/srcLanTask1/exercise4/scripts/haarcascade_frontalface_default.xml')
    faces = face_cascade.detectMultiScale(img, scaleFactor=1.01, minNeighbors=10)
    faces_wanted = face_cascade.detectMultiScale(wantedImg, scaleFactor=1.01, minNeighbors=7)
    print('tu')

    offset = 10
    for (x, y, w, h) in faces:
        # cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 3)
        adjusted = img[y-offset:y+h+offset,x-offset:x+w+offset]

        # alpha = 3  # contrast:  [0,1) => lower    [1,] => higher
        # beta = 2   # brightness
        # adjusted = cv2.convertScaleAbs(adjusted, alpha, beta)
        height, width = adjusted.shape[:2]
        size = height * width
        cv2.imshow('face', adjusted)
        cv2.waitKey(0)

        for(x2, y2, w2, h2) in faces_wanted:
            print('tu')
            adjusted2 = wantedImg[y2-offset:y2+h2+offset,x2-offset:x2+w2+offset]
            height2, width2 = adjusted2.shape[:2]
            size2 = height2 * width2
            cv2.imshow('face wanted', adjusted2)
            cv2.waitKey(0)
            print('tu')
            if size < size2: #
                adjusted = cv2.resize(adjusted, (width2, height2), interpolation=cv2.INTER_CUBIC)
            else:
                adjusted2 = cv2.resize(adjusted2, (width, height), interpolation=cv2.INTER_CUBIC)
            
            # embedder = FaceNet()
            # print('starting embed')
            # embedding1 = embedder.embeddings(adjusted)
            # print('embed 1')
            # embedding2 = embedder.embeddings(adjusted2)
            # print('embed 2')

            distance = np.linalg.norm(embedding1 - embedding2)

            print(distance)

    return

# Paths to the input images
image1_path = '/home/nana/ROS/src/task2/meshes/neki3.png'
image2_path = '/home/nana/ROS/src/task2/meshes/wanted1.png'

# Compare the faces and get the similarity score
similarity_score = compare_faces(image1_path, image2_path)

# Print the similarity score
print("Similarity score: ", similarity_score)

exit(1)

import cv2
import face_recognition
import numpy as np

img = cv2.cvtColor(cv2.imread('/home/nana/ROS/src/task2/meshes/neki3.png'), cv2.COLOR_BGR2GRAY)
wantedImg = cv2.cvtColor(cv2.imread('/home/nana/ROS/src/task2/meshes/wanted1.png'), cv2.COLOR_BGR2GRAY)

face_cascade = cv2.CascadeClassifier('/home/nana/ROS/srcLanTask1/exercise4/scripts/haarcascade_frontalface_default.xml')
faces = face_cascade.detectMultiScale(img, scaleFactor=1.01, minNeighbors=10)
faces_wanted = face_cascade.detectMultiScale(wantedImg, scaleFactor=1.01, minNeighbors=7)
hog = cv2.HOGDescriptor()

# f = [img[y-offset:y+h+offset,x-offset:x+w+offset] for (x, y, w, h) in faces]
# fw = [wantedImg[y-offset:y+h+offset,x-offset:x+w+offset] for (x, y, w, h) in faces_wanted]

offset = 10
for (x, y, w, h) in faces:
    # cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 3)
    adjusted = img[y-offset:y+h+offset,x-offset:x+w+offset]

    # alpha = 3  # contrast:  [0,1) => lower    [1,] => higher
    # beta = 2   # brightness
    # adjusted = cv2.convertScaleAbs(adjusted, alpha, beta)
    height, width = adjusted.shape[:2]
    size = height * width
    cv2.imshow('face', adjusted)
    cv2.waitKey(0)

    for(x2, y2, w2, h2) in faces_wanted:
        adjusted2 = wantedImg[y2-offset:y2+h2+offset,x2-offset:x2+w2+offset]
        height2, width2 = adjusted2.shape[:2]
        size2 = height2 * width2
        cv2.imshow('face wanted', adjusted2)
        cv2.waitKey(0)

        if size < size2: #
            adjusted = cv2.resize(adjusted, (width2, height2), interpolation=cv2.INTER_CUBIC)
        else:
            adjusted2 = cv2.resize(adjusted2, (width, height), interpolation=cv2.INTER_CUBIC)
        
        features1 = hog.compute(adjusted).flatten()
        features2 = hog.compute(adjusted2).flatten()

        dot_product = np.dot(features1, features2)
        norm_1 = np.linalg.norm(features1)
        norm_2 = np.linalg.norm(features2)
        similarity = dot_product / (norm_1 * norm_2)

        print("Histogram Similarity: ", similarity)
        cv2.destroyAllWindows()
    # histogram1 = cv2.calcHist([adjusted], [0], None, [256], [0, 256])
    # histogram2 = cv2.calcHist([wantedImg], [0], None, [256], [0, 256])

    # # Normalize the histograms (optional)
    # histogram1 = cv2.normalize(histogram1, histogram1, 0, 1, cv2.NORM_MINMAX)
    # histogram2 = cv2.normalize(histogram2, histogram2, 0, 1, cv2.NORM_MINMAX)

    # # Calculate the histogram similarity using the Bhattacharyya coefficient
    # similarity = cv2.compareHist(histogram1, histogram2, cv2.HISTCMP_BHATTACHARYYA)


exit(0)





def get_3d_rotation_matrix(width, height, theta, phi, gamma, dx = 0, dy = 0, dz = 0):
    w, h = width, height
    d = np.sqrt(w ** 2 + h ** 2)
    focal = f = d / (2 * np.sin(gamma) if np.sin(gamma) != 0 else 1)
    dz = focal

    # Projection 2D -> 3D matrix
    A1 = np.array([[1, 0, -w / 2],
                   [0, 1, -h / 2],
                   [0, 0, 1],
                   [0, 0, 1]])

    # Rotation matrices around the X, Y, and Z axis
    RX = np.array([[1, 0, 0, 0],
                   [0, np.cos(theta), -np.sin(theta), 0],
                   [0, np.sin(theta), np.cos(theta), 0],
                   [0, 0, 0, 1]])

    RY = np.array([[np.cos(phi), 0, -np.sin(phi), 0],
                   [0, 1, 0, 0],
                   [np.sin(phi), 0, np.cos(phi), 0],
                   [0, 0, 0, 1]])

    RZ = np.array([[np.cos(gamma), -np.sin(gamma), 0, 0],
                   [np.sin(gamma), np.cos(gamma), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    # Composed rotation matrix with (RX, RY, RZ)
    R = np.dot(np.dot(RX, RY), RZ)

    # Translation matrix
    T = np.array([[1, 0, 0, dx],
                  [0, 1, 0, dy],
                  [0, 0, 1, dz],
                  [0, 0, 0, 1]])

    # Projection 3D -> 2D matrix
    A2 = np.array([[f, 0, w / 2, 0],
                   [0, f, h / 2, 0],
                   [0, 0, 1, 0]])

    # Final transformation matrix
    return np.dot(A2, np.dot(T, np.dot(R, A1)))

img = cv2.imread('/home/nana/ROS/src/task2/meshes/neki4.png')
wantedImg = cv2.imread('/home/nana/ROS/src/task2/meshes/robber3240if0.jpg')

height, width = img.shape[:2]
mat = get_3d_rotation_matrix(height, width, 1, 0, 0)
img = cv2.warpPerspective(img, mat, (height, width))#, cv2.INTER_CUBIC | cv2.WARP_INVERSE_MAP)

face_cascade = cv2.CascadeClassifier('/home/nana/ROS/srcLanTask1/exercise4/scripts/haarcascade_frontalface_default.xml')
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
faces = face_cascade.detectMultiScale(img, scaleFactor=1.01, minNeighbors=10)

rng = 36
offset = 10

for (x, y, w, h) in faces:
    # cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 3)
    adjusted = img[y-offset:y+h+offset,x-offset:x+w+offset]

    alpha = 3  # contrast:  [0,1) => lower    [1,] => higher
    beta = 2   # brightness
    adjusted = cv2.convertScaleAbs(adjusted, alpha, beta)
    height, width = adjusted.shape[:2]

    for i in range(rng):
        angle = i * 10
        rot_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), angle, 1.0)
        rot_img = cv2.warpAffine(adjusted, rot_matrix, (width, height))

        cv2.imshow('img', rot_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        camera_encoding = face_recognition.face_encodings(rot_img)
        if len(camera_encoding) != 0:
            camera_encoding = camera_encoding[0]
            #print(camera_encoding)
            cv2.imshow('Face Detection ' + angle, rot_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print(angle, 'nism nausu sm faco')
    # scale_factor = 2
    # adjusted = cv2.resize(cropped, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_CUBIC)


    # wanted = face_recognition.face_encodings(wantedImg)[0]

    # results = face_recognition.compare_faces([wanted], camera_encoding)
    # print(results[0])




