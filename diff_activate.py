import cv2
import numpy as np

# video = cv2.VideoCapture('ring_pretzle_test.avi')
video = cv2.VideoCapture(0)

i = 0  # video file name incrementer
frame = np.array([])
prev_frame = np.array([])
write = False
init = False
fourCC = cv2.VideoWriter_fourcc(*'MPEG')
while 1:
    prev_frame = frame
    ret, frame = video.read()
    if not ret:
        break

    if not init:
        prev_frame = frame
        init = True

    diff = cv2.absdiff(frame, prev_frame)
    cv2.imshow('Window', diff)

    layer = diff[:,:,0]  # Take top layer of diff (RGB values are equal, so only need one layer)
    thresh_intensity = 80  # threshold intensity value (0-255)
    thresh_size = 30       # number of pixels needed to be above thresh_intensity
    row = 20               # row number (from top) to determine if object is fully in frame

    if layer[layer>thresh_intensity].size > thresh_size \
        and np.all(layer[row]<thresh_intensity):

        ################# PLACE OBJECT DETECTION CODE BELOW ######################

        print("ACTIVATE")

        ################# PLACE OBJECT DETECTION CODE ABOVE ######################

        key = cv2.waitKey(50) # slow down video feed for object detection
    else:
        key = cv2.waitKey(5)


    # Press 'q' to exit, 'w' to start writing to file then 'w' again to save file
    if key == ord('q'):
        break
    elif key == ord('w'):
        if not write:
            write = True
            out = cv2.VideoWriter('video_{}.avi'.format(i),fourCC,25.0,(640,480))
            print('Recording video...')
        else:
            write = False
            out.release()
            print('video_{}.avi saved!'.format(i))
            i += 1

    if write:
        out.write(diff)

video.release()
cv2.destroyAllWindows()
