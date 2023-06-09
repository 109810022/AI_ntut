#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
from math import copysign
import time
from shapely import Polygon

ROI = [[0,0],[640,240]]
IOU = [None,None]
reg =["0","0"]
#insert = "0"
reg_dict = {
    "milktea":0,
    "redtea":0,
    "greentea":0,
}
index2item = [ "milktea","redtea","greentea"]
# Load the YOLOv8 model
model = YOLO('yolov8n.pt')
# Open the video file
video_path = 0 #"path/to/your/video/file.mp4"
cap = cv2.VideoCapture(video_path)

 #init
frames_num =10

#data structure --> stack
def push(reg,push_item):
    reg.pop()
    reg.insert(0,push_item)
    return reg

#to get IOU
def get_iou(obj,ROI = [(0,0),(640,0),(640,240),(0,240)]): #obj needs array of xyxy in result ; we can set ROI by ourself  
    roi_region = Polygon(ROI)
    item_region = Polygon([[obj[0],obj[1]],[obj[0],obj[3]],[obj[2],obj[3]],[obj[2],obj[1]]])
    iou = item_region.intersection(roi_region).area / item_region.area
    #print(iou)
    return iou

#whether the thing in or not
def in_or_out(IOU):
    if None not in IOU:
                if IOU[1]>=IOU[0]:
                    print("biggggggger")
                    IOU = [None,None]
                    #pub.publish("")
                    return 1 # 進入加一

                elif IOU[1]<IOU[0]:
                    print("smallllller")
                    IOU = [None,None]
                    #pub.publish("")
                    return -1 #取出減一
    return 0

def shop_list(sign, item):
    print("hello world")
    reg_dict[item]=reg_dict[item]+sign
    print(reg_dict)
     
def main():
    frames = 0
    # Loop through the video frames
    while cap.isOpened():
        # Read a frame from the video
        success, frame = cap.read()
        frames = frames+1
        
        if success:
            # Run YOLOv8 inference on the frame
            results = model(frame,conf=0.5)
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
                iou = get_iou(boxes.xyxy[-1],)
                push(IOU,iou)
                sign = in_or_out(IOU)
                print(sign)
                print(int(boxes.cls[-1]))
                shop_list(sign,index2item[int(boxes.cls[-1])])
                

        except:
            pass
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        #shop_list(1,index2item[0])
    # Release the video capture object and close the display window
    cap.release()
    cv2.destroyAllWindows()

def final_list(f_list):
    print("*************************")
    pass

if __name__ == "__main__":
    main()
    final_list(reg_dict)

