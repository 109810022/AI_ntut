#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
import rospy
from std_msgs.msg import String
from roslib import message
from geometry_msgs.msg import Twist
from math import copysign
import time
from shapely import Polygon

ROI = [[0,0],[640,240]]
IOU = [None,None]
puber = "/shopping" 
# Load the YOLOv8 model
model = YOLO('best.pt')
# Open the video file
video_path = 0 #"path/to/your/video/file.mp4"
cap = cv2.VideoCapture(video_path)
pub = rospy.Publisher(puber, String, queue_size=10)
rospy.init_node('talker', anonymous=True)
frames = 0 #init
frames_num =1


#to get IOU
def get_iou(obj,ROI = [(0,0),(640,0),(640,240),(0,240)]): #obj needs array of xyxy in result ; we can set ROI by ourself  
    roi_region = Polygon(ROI)
    item_region = Polygon([[obj[0],obj[1]],[obj[0],obj[3]],[obj[2],obj[3]],[obj[2],obj[1]]])
    iou = item_region.intersection(roi_region).area / item_region.area
    #print(iou)
    return iou

def push(reg,push_item):
    reg.pop()
    reg.insert(0,push_item)
    return reg

def in_or_out(IOU):
    if None not in IOU:
                if IOU[1]>=IOU[0]:
                    rospy.loginfo(str(IOU))
                    rospy.loginfo("in")
                    IOU = [None,None]
                    #pub.publish("")
                    pass
                elif IOU[1]<IOU[0]:
                    rospy.loginfo(str(IOU))
                    rospy.loginfo("out")
                    IOU = [None,None]
                #pub.publish("")
                    pass



# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()
    frames = frames+1
    if success:
        
        # Run YOLOv8 inference on the frame
        results = model(frame,conf=0.7)
        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        boxes = results[0].boxes
        cv2.rectangle(annotated_frame, ROI[0], ROI[1], (0, 255, 0), 2)
    
        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

    else:
        # Break the loop if the end of the video is reached
        break
    
    try:
        if frames==frames_num:
            frames = 0
            iou = get_iou(boxes.xyxy[0],)
            push(IOU,iou)
            in_or_out(IOU)
            
            
    except:
        pass
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    
# Release the video capture object and close the display window

cap.release()
cv2.destroyAllWindows()
rospy.loginfo("shutdown")
pub.publish("shutdown")
