#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
from math import copysign
import time
from shapely import Polygon
import  pandas as pd
import csv



ROI = [[0,0],[640,240]]
IOU = [None,None]
reg =[None,None]
button_click = False
reg_dict = {
    "milktea":0,
    "redtea":0,
    "greentea":0,
}
price = {
    "redtea":10,
    "milktea":20,
    "greentea":30,
}
index2item = ["redtea","milktea","greentea"]

# Load the YOLOv8 model
model = YOLO('best.pt')
# Open the video file
video_path = 0 #"path/to/your/video/file.mp4"
cap = cv2.VideoCapture(video_path)

#init
frames_num =10
total_price = 0
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
        inout = IOU[1] - IOU[0]
        if inout>0:
            IOU = [None,None]
            #pub.publish("")
            return 1 # 進入加一

        elif inout<0:
            IOU = [None,None]
            #pub.publish("")
            return -1 #取出減一
    return 0

def shop_list(sign, item):
    reg_dict[item]=reg_dict[item]+sign
    print(reg_dict)
        
def final_list(f_list):
    
    df = pd.DataFrame([reg_dict,price],).T
    df.columns = ['num', 'value',]
    df["price"]=df["num"]*df["value"]
    print("*************************")
    pd.set_option('display.unicode.ambiguous_as_wide', True)
    pd.set_option('display.unicode.east_asian_width', True)
    pd.set_option('display.width', 180)                       # 设置打印宽度(**重要**)
    print(df)
    print("*************************")
    print("                   total:"+str(sum(df["price"])))
    df.to_csv("total_price.csv", encoding = 'utf_8_sig',)
    global total_price
    total_price = sum(df["price"])

def button_event(event,x,y,flags,userdata):
    if event == 1 : 
        #if x in range(670,870) and y in (350,450) :
        if 700<x<840 and 350<y<430:
            print("pay!!!")
            global button_click 
            button_click = True
        
def show(img):
    with open("total_price.csv","r",encoding="utf_8") as csvFile : #開啟檔案
        csvReader = csv.reader(csvFile) #將檔案建立成Reader物件
        listReport = list(csvReader) #將資料轉成list(串列)

    cv2.rectangle(img, ROI[0], ROI[1], (0, 255, 0), 2)
    img = cv2.copyMakeBorder(img, 0, 0, 0, 300,0,img, [255,255,255])
    for i , row in enumerate(listReport):
        for j,item in enumerate(row):
            if i==0 and j==0:
                cv2.putText(img,"type" ,[650+j*80,30+i*20],  cv2.FONT_HERSHEY_TRIPLEX, 0.5, 0)
            else:
                cv2.putText(img,item ,[650+j*80,30+i*20],  cv2.FONT_HERSHEY_TRIPLEX, 0.5, 0)
    cv2.rectangle(img, [700,350],[840,430] , (0, 0, 0), 2)
    cv2.putText(img,"PAY" ,[720,407],  cv2.FONT_HERSHEY_TRIPLEX, 1.5, 0)
    cv2.putText(img,"total price:"+str(total_price),[700,300],  cv2.FONT_HERSHEY_TRIPLEX, .5, 0)
    return img

while True:
    motion = [None,None]
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
            annotated_frame = show(annotated_frame)
            # Display the annotated frame
            cv2.imshow("YOLOv8 Inference", annotated_frame)
        else:
            # Break the loop if the end of the video is reached
            break

        if frames==frames_num:
            frames = 0
            try:
                iou = get_iou(boxes.xyxy[-1],)
                push(IOU,iou)
                
                sign = in_or_out(IOU)
                push(motion,[sign,index2item[int(boxes.cls[-1])]])
                if motion[0]!=motion[1]:
                    shop_list(sign,index2item[int(boxes.cls[-1])])
                    totalprice = final_list(reg_dict)
                
            except:
                IOU = [None,None]
                reg =[None,None]
                motion = [None,None]
                pass
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q") or button_click == True :
            break
        cv2.setMouseCallback("YOLOv8 Inference", button_event)
        
    break
        #shop_list(1,index2item[0])
    # Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()


    





