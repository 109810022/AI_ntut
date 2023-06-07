#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
import rospy
from std_msgs.msg import String
from roslib import message
from geometry_msgs.msg import Twist, TwistStamped
from math import copysign
import time
from shapely import Polygon

ROI = [[0,0],[240,640]]
IOU = [None,None]
puber = "/shopping" 
# Load the YOLOv8 model
model = YOLO('yolov8n.pt')
# Open the video file
video_path = 0 #"path/to/your/video/file.mp4"
cap = cv2.VideoCapture(video_path)
pub = rospy.Publisher(puber, String, queue_size=10)
rospy.init_node('talker', anonymous=True)
frames = 0 #init
frames_num =10


#to get IOU
def get_iou(obj,ROI = [(0,0),(0,640),(240,640),(240,0)]): #obj needs array of xyxy in result ; we can set ROI by ourself  
    roi_region = Polygon(ROI)
    item_region = Polygon([[obj[0],obj[1]],[obj[0],obj[3]],[obj[2],obj[3]],[obj[2],obj[1]]])
    iou = item_region.intersection(roi_region).area / item_region.area
    #print(iou)
    return iou

def push(reg,push_item):
    reg.pop()
    reg.insert(0,push_item)
    return reg

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        frames = frames+1
        # Run YOLOv8 inference on the frame
        results = model(frame,conf=0.7)
        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        boxes = results[0].boxes
    
        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break
    if frames==frames_num:
        iou = get_iou(boxes.xyxy[-1],)
        push(IOU,iou)
        if None not in IOU:
            if IOU[1]>=IOU[0]:
                rospy.loginfo("in")
                IOU = [None,None]
                #pub.publish("")
                pass
            elif IOU[1]<IOU[0]:
                rospy.loginfo("out")
                IOU = [None,None]
            #pub.publish("")
                pass
        frames = 0
        pass

# Release the video capture object and close the display window

cap.release()
cv2.destroyAllWindows()
rospy.loginfo("shutdown")
pub.publish("shutdown")
