import cv2, time
from djitellopy import Tello
from all_functions import *
from threading import Thread

#global variables
for_back_velocity = left_right_velocity = up_down_velocity = yaw_velocity = 0

pid_up_down = [0.4,2,0]
pid_yaw = [0.2,0.5,0]
pid_front_back = [1,1.6,0]

video_fps = 60
frame_w = 720
frame_h = 480

cam_center_x = int(frame_w/2)
cam_center_y = int(frame_h/2)
ideal_w = 100
dead_zone_side = 5

x_error = 0.0
y_error = 0.0
z_error = 0.0
p_x_error = 0.0
p_y_error = 0.0
p_z_error = 0.0
p_center_x = 0
p_center_y = 0
false_positive_radius = 50
p_found_face = False 
found_face = False

#init
tello = Tello()
tello.connect()
tello.streamon()
tello.takeoff()
time.sleep(4)
tello.move_up(90)
video = cv2.VideoWriter('footage/trip_test_face_2.avi', cv2.VideoWriter_fourcc(*'XVID'), video_fps, (frame_w, frame_h))
face_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_frontalface_default.xml')

while True:
    #getting and editing the image
    img = tello.get_frame_read().frame
    img = cv2.flip(img,1)
    img = cv2.resize(img, (frame_w, frame_h))

    #find faces
    face_rects = face_cascade.detectMultiScale(img, scaleFactor=1.1, minNeighbors=4) 

    #drawing
    cv2.circle(img,(cam_center_x,cam_center_y),10,(0,0,255),4)
    cv2.putText(img,str(tello.get_battery()),(10,frame_h-20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0))

    if len(face_rects) > 0: 
        found_face = True
    else:
        found_face = False

    if found_face and p_found_face:
        x, y, face_w, face_h = face_rects[0]
        (face_center_x, face_center_y) = get_face_center(x, y, face_w, face_h)
        
        found_face = is_face(face_center_x, face_center_y, p_center_x, p_center_y, false_positive_radius)
        
        if not found_face: cv2.putText(img, "False Positive", (40, frame_h-230), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)

    p_found_face = found_face

    #no face detected
    if not found_face:
        cv2.putText(img, "Failure to Detect Tracking!", (40,frame_h-20), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),1)

        #do nothing
        for_back_velocity = left_right_velocity = up_down_velocity = yaw_velocity = 0

    #faces detected
    else:
        #get coordinates of face
        x, y, face_w, face_h = face_rects[0]
        (face_center_x, face_center_y) = get_face_center(x, y, face_w, face_h)
        cv2.rectangle(img, (x, y), (x+face_w, y+face_h), (255, 0, 0), 2)
        #cv2.circle(img,(face_center_x,face_center_y),false_positive_radius,(50,50,200),3)
        
        x_error = face_center_x - cam_center_x
        y_error = face_center_y - cam_center_y
        z_error = face_w - ideal_w
        if z_error > dead_zone_side:
            z_error = z_error - dead_zone_side
        elif z_error < - dead_zone_side:
            z_error = z_error + dead_zone_side
        else:
            z_error = 0

        cv2.putText(img, "x_error:" + str(x_error), (20,frame_h-45), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),1)
        cv2.putText(img, "y_error:" + str(y_error), (20,frame_h-70), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),1)
        cv2.putText(img, "z_error:" + str(z_error), (20, frame_h-95),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)

        #cv2.putText(img, "faces_detected:" + str(face_rects.size), (20, frame_h-200),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
        

        #cv2.rectangle(img, (int(face_center_x-ideal_w/2-dead_zone_side), int(face_center_y-ideal_w/2-dead_zone_side)),
        #(int(face_center_x+ideal_w/2+dead_zone_side), int(face_center_y+ideal_w/2+dead_zone_side)), (255,100,100), 2)

        #cv2.rectangle(img, (int(face_center_x-ideal_w/2+dead_zone_side), int(face_center_y-ideal_w/2+dead_zone_side)),
        #(int(face_center_x+ideal_w/2-dead_zone_side), int(face_center_y+ideal_w/2-dead_zone_side)), (255, 100, 100), 2)

        cv2.circle(img,(face_center_x,face_center_y),10,(255,0,0),4)
        cv2.line(img,(cam_center_x,cam_center_y),(face_center_x,face_center_y),(0,255,0),3)
        
        for_back_velocity = set_z_vel(z_error, p_z_error, pid_front_back) 
        
        up_down_velocity = set_y_vel(y_error, p_y_error, pid_up_down)

        yaw_velocity = set_yaw_vel(x_error, p_x_error, pid_yaw)
        
        p_x_error = x_error
        p_y_error = y_error
        p_z_error = z_error
        p_center_x = face_center_x
        p_center_y = face_center_y

    cv2.putText(img, "up_down:" + str(up_down_velocity), (20,frame_h-120), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),1)
    cv2.putText(img, "for_back:" + str(for_back_velocity), (20,frame_h-145), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),1)
    cv2.putText(img, "yaw:" + str(yaw_velocity), (20, frame_h-170),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)

    tello.send_rc_control(-left_right_velocity, -for_back_velocity, -up_down_velocity, -yaw_velocity)
    
    show_and_save(img, video)

    if cv2.waitKey(1) & 0xFF == ord('l'):
        break

tello.streamoff()
video.release()
print('LOOP ENDED')
tello.land()

        
