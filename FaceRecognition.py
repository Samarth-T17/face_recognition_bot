import face_recognition
import cv2
import numpy as np
import subprocess
import serial
import time

port = '/dev/ttyACM0' 
baud_rate = 9600  
# mode='5'
# time.sleep(3)
ser = serial.Serial(port, baud_rate)


def linear_search(namesmy, key):
    for i, name in enumerate(namesmy):
        if name == key:
            return i
    return -1


bash_script = "/home/samarth/Documents/nice.sh"

# Define the input string
k=0
time_3o=0
time_1=0
mid=320

video_capture = cv2.VideoCapture(2)
top_lo=[0,0]
top_ln=[0,0]

samarth_image = face_recognition.load_image_file("/home/samarth/Documents/expo/Samarth.jpg")
samarth_face_encoding = face_recognition.face_encodings(samarth_image)[0]

Ashwin_image = face_recognition.load_image_file("/home/samarth/Documents/expo/Ashwin.jpg")
Ashwin_face_encoding = face_recognition.face_encodings(Ashwin_image)[0]


known_face_encodings = [
    samarth_face_encoding,
    Ashwin_face_encoding
]
known_face_names = [
    "Samarth",
    "Ashwin"
]
namesmy=["Samarth","Ashwin"]
nfps=30
ifps=0
data='1'
ser.write(data.encode())
# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
process_this_frame = True
#ser.write(mode.encode())
while True:

    ret, frame = video_capture.read()
    frame = cv2.flip(frame, 1)
    temp=frame;
    if process_this_frame:
        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

        face_locations = face_recognition.face_locations(small_frame)
        face_encodings = face_recognition.face_encodings(small_frame, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
            name = "unknown"

            # if True in matches:
            #     first_match_index = matches.index(True)
            #     name = known_face_names[first_match_index]

            face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = known_face_names[best_match_index]

            key = name 

            index = linear_search(namesmy, key)
            if index != -1:
                save_path = '/home/samarth/Documents/expophotos/intruder.jpg'
                # Save the frame to the specified path
                #cv2.imwrite(save_path, temp)
                time_1=time.time()
                data='5'
                ser.write(data.encode())
                #process = subprocess.Popen(['bash', bash_script], stdin=subprocess.PIPE)
                #process.communicate(input=key.encode())
                namesmy[index] = ""
            
            face_names.append(name)

    process_this_frame = not process_this_frame

    for (top, right, bottom, left), name in zip(face_locations, face_names):
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4
        ifps=ifps+1
        mid=(left+right)/2
        print(mid)
        # k=1
        # ifps=ifps+1
        # if ifps==nfps:
        #     ifps=0
        #     print(mid)
        #     if mid>320:
        #         data='d'
        #         print(data)
        #         ser.write(data.encode())
        #     if mid<320:
        #         data='a'
        #         print(data)
        #         ser.write(data.encode())
        # # Draw a box around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)


        # Draw a label with a name below the face
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
        #time.sleep(0.294)
    # Display the resulting image
    cv2.imshow('Video', frame)
    
    ifps=ifps+1
    if ifps==nfps:
        ifps=0
        print(mid)
        if mid>384:
            data='d'
            print(data)
            ser.write(data.encode())
        if mid<256:
            data='a'
            print(data)
            ser.write(data.encode())

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):  # Check for 'q' key press
        break
    elif key != 255:
        ser.write(chr(key).encode()) 


video_capture.release()
cv2.destroyAllWindows()
