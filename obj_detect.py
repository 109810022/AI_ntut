import cv2
from ultralytics import YOLO
import time
from shapely import Polygon
import numpy as np
import pandas as pd
# Load the YOLOv8 model
model = YOLO('yolov8n.pt')
#定義ROI範圍
ROI = [[100,100],[400,400]]
##### 紀錄購買的商品數量
number = {}
number = 0
##### 定義商品種類數量及價目表


def get_iou(obj,ROI = [(100,100),(100,400),(400,400),(400,100)]): #obj needs array of xyxy in result ; we can set ROI by ourself  
    roi_region = Polygon(ROI)
    item_region = Polygon([[obj[0],obj[1]],[obj[0],obj[3]],[obj[2],obj[3]],[obj[2],obj[1]]])
    iou = item_region.intersection(roi_region).area / item_region.area
    return iou

def make_invoice(type , iou , dict1 ): 
    #type is the type of product ; iou in 0.65 ~ 0.7 -> purchase ; dict is the product information
    if   0.65< iou < 0.7 :
        try :
            dict1[type]= dict1[type] + 1
        except :
            dict1 = dict1 + 1    
    return dict1
     

# Open the video file
video_path = 0 
cap = cv2.VideoCapture(video_path)

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        results = model(frame,conf=0.7)

        # Visualize the results and ROI on the frame 
        annotated_frame = results[0].plot()
        annotated_frame = cv2.rectangle(annotated_frame, ROI[1], ROI[0], (0,255,0), 2) #show the roi
        
        #calculate the number of product
        for num , item in enumerate(results[0].boxes.cls):
            iou = get_iou(results[0].boxes.xyxy.numpy()[num])
            if item  == 0:
                print(str(item)+":" + str(iou)+"\n")
                number = make_invoice(item , iou , number)
                print(number)
            #number = make_invoice(obj_dict[item] , iou , number)
            
        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()

print(number)
