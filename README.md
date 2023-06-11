# AI_ntut final project
人工智慧與工程應用實做期末專題--> **智能結帳購物車**  
我們使用ROS系統結合yolov8的影像辨識，達成可以在將物品放入購物車的同時進行儲存,並發送訊號進行結帳，使得消費者可以減少購物結帳的時間 。 
而且在未來將數據收集起來，結合大數據分析每個消費者的購買習慣，制定適合消費者的優惠方案。亦可結合商品的銷售量來取得最佳的進貨量。  
# 原理
物體辨識:利用ultralytivs 提供之API，獲得每個物件的座標  
取出&放入: 利用計算IOU的方式，辨識前一幀和後一幀物體和一定範圍的交集面積，若增加-->取出，減少-->放入  

# shopping_list.py + yolo_shopping.py
## 使用方法
將程式放入ROS的workspace，並執行:    
``` source devel/setup.bash ```  
先啟動 shopping_list  
``` rosrun beginner_tutorials shopping_list.py ```  
再啟動  yolo_shopping  
``` rosrun beginner_tutorials   yolo_shopping.py ```  
## 軟體配置  
Ubuntu 18.04  
ROS melodic 1.14.13  
shopping_list.py --> python 2.7   
yolo_shopping.py --> python=3.9   
## 流程  
※將影像辨識到且滿足條件的物體，由```/talker``` 節點以```/shopping```的topic發布消息，並由```/listerner```節點接收並儲存購買的物件。  
※要結帳時，透過關閉辨識視窗，```/talker```會發布內容為"shutdown"的消息給```/listerner```，```shopping_list.py```會輸出你買的物品明細及價錢

#v2是基本的算法及大概的可視畫

# v4shoppingcart --> 新增"偽"介面
## --> for win
不須使用ros環境即可執行，和上面的原理相同

# 使用函式庫
ultralytics  
cv2  
shapely

