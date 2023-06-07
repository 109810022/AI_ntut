import cv2
from ultralytics import YOLO
import time
from shapely import Polygon
import numpy as np
ROI = [[100,100],[400,400]]
img= "C:\\Users\\cchhi\\Desktop\\yolov8\\human.jpg"
model = YOLO('yolov8n.pt')
#to get IOU
def get_iou(obj,ROI = [(100,100),(100,400),(400,400),(400,100)]): #obj needs array of xyxy in result ; we can set ROI by ourself  
    roi_region = Polygon(ROI)
    item_region = Polygon([[obj[0],obj[1]],[obj[0],obj[3]],[obj[2],obj[3]],[obj[2],obj[1]]])
    iou = item_region.intersection(roi_region).area / item_region.area
    #print(iou)
    return iou
    



results = model(img)
annotated_frame = results[0].plot() #plot the item
annotated_frame = cv2.rectangle(annotated_frame, ROI[1], ROI[0], (0,255,0), 2) #show the roi
cv2.imshow("YOLOv8 Inference", annotated_frame) #show frame

# to get every item's IOU
for num , item in enumerate(results[0].boxes.cls):
    print(str(results[0].boxes.xyxy.numpy()[num])+"\n")
    a = get_iou(results[0].boxes.xyxy.numpy()[num])
    print(str(a)+"\n")


cv2.waitKey(0)
cv2.destroyAllWindows()
