#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time
import csv

reg =["0","0"]
insert = "0"
reg_list = {
"milktea":0,
"redtea":0,
"greentea":0,
}
def push(reg,push_item):
    reg.pop()
    reg.insert(0,push_item)
    return reg


def callback(data): ## define way here
    global insert 
    insert = data.data
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
 
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/shopping", String, callback)
     # spin() simply keeps python from exiting until this node is stopped


 
if __name__ == '__main__':
    start = True
    while start:
        listener()
        reg = push(reg , insert)
        if reg[0] == "shutdown":
            start = False
            print("*****************************")
            print("你的收據:")
            print(reg_list)
            print("*****************************")

        elif reg[0]!=reg[1]:
            print(reg)
            print(insert)
            reg_list[reg[0]] = reg_list[reg[0]]+1
            print(reg_list)
        
        


        #try:
            #time.sleep(2)
        #except KeyboardInterrupt:
            #start = False

      
