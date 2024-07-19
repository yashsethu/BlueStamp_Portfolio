from picamera2 import Picamera2
import numpy as np
import cv2
import time

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (320, 240)}))
picam2.start()
time.sleep(2)

while True:
    frame = picam2.capture_array()
    
    if frame is None:
        print("Error: Frame not captured")
        break
    
    cv2.imshow("Tracking", frame)

    if(cv2.waitKey(1) & 0xff == ord('q')): #Press q to break the loop and stop moving 
        break

cv2.destroyAllWindows()
picam2.stop()
