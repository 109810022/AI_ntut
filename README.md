# AI_ntut final project
人工智慧與工程應用實做期末專題--> **智能結帳購物車**  
我們使用yolov8的影像辨識，達成可以在將物品放入購物車的同時進行儲存,並進行結帳，使得消費者可以減少購物結帳的時間 。 
而且在未來將數據收集起來，結合大數據分析每個消費者的購買習慣，制定適合消費者的優惠方案。亦可結合商品的銷售量來取得最佳的進貨量。  
### 新方法 -> https://github.com/109810022/2023_innovated_race  


# 原理
物體辨識:利用ultralytivs 提供之API，獲得每個物件的座標  
取出&放入: 利用計算IOU的方式，辨識前一幀和後一幀物體和一定範圍的交集面積，若增加-->取出，減少-->放入  
# 最終版本 ==> UI+shoppingcart  

## 軟體配置  
win11  
python 3.9.16  
# v4shoppingcart  
新增"偽"介面
![螢幕擷取畫面 2023-06-11 150654](https://github.com/109810022/AI_ntut/assets/100888502/d54395f0-ca1c-4634-97a0-19dcb38ad2f5)


# 使用函式庫
ultralytics  
cv2  
shapely  
pandas  
csv  
time  


# 結合ROS =>跑太慢了 放棄xdd  
## shopping_list.py + yolo_shopping.py
### 使用方法
將程式放入ROS的workspace，並執行:    
``` source devel/setup.bash ```  
先啟動 shopping_list  
``` rosrun beginner_tutorials shopping_list.py ```  
再啟動  yolo_shopping  
``` rosrun beginner_tutorials   yolo_shopping.py ```  
### 軟體配置  
Ubuntu 18.04  
ROS melodic 1.14.13  
shopping_list.py --> python 2.7   
yolo_shopping.py --> python=3.9   
### 流程  
※將影像辨識到且滿足條件的物體，由```/talker``` 節點以```/shopping```的topic發布消息，並由```/listerner```節點接收並儲存購買的物件。  
※要結帳時，透過關閉辨識視窗，```/talker```會發布內容為"shutdown"的消息給```/listerner```，```shopping_list.py```會輸出你買的物品明細及價錢




