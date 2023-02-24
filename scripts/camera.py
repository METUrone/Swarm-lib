#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import rospy
from swarm.srv import FireData

rospy.init_node("redgrid_client",anonymous=True)
service_name="firedata"

class Red_object:
    def __init__(self,number_of_division=20, path = "test.png"):
        self.division_number=number_of_division
        self.firerow=np.array([[]])
        self.first_row=None
        self.path=path
    def bluring(self):
        self.blured=cv.blur(self.frame,(9,9))
    def converting_to_hsv(self):
        self.hsv=cv.cvtColor(self.blured,cv.COLOR_BGR2HSV)
    def creating_mask(self):
        self.lower1=np.array([160,50,50])
        self.upper1=np.array([180,255,255])
        self.lower2=np.array([0,50,50])
        self.upper2=np.array([20,255,255])
        self.mask1=cv.inRange(self.hsv,self.lower1,self.upper1)
        self.mask2=cv.inRange(self.hsv,self.lower2,self.upper2)
        self.mask=cv.bitwise_or(self.mask1,self.mask2)
        self.masking_result=cv.bitwise_and(self.frame,self.frame,mask=self.mask)
    def dilation(self):
        self.kernel=np.ones((15,15),dtype=np.uint8)
        self.dilated=cv.dilate(self.mask,self.kernel,iterations=1)
    def detecting_contours(self):
        self.contours,self.hiearchy=cv.findContours(self.square,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)
        for i in self.contours:
            a=cv.contourArea(i)
            if a>0:
                return True
    def dividing_screen(self):
        self.shape=self.frame.shape
        self.height=self.shape[0]
        self.width=self.shape[1]
        self.square_height=int(self.height/self.division_number)
        self.square_width=int(self.width/self.division_number)
        self.number_of_squares=self.division_number**2
        self.list_of_centers=[]
        y0=0
        x0=0
        for y in range(self.division_number):
            x0=0
            self.firerow=np.array([[]])
            for x in range(self.division_number):
                self.square=self.dilated[y0:y0+self.square_height,x0:x0+self.square_width:]
                self.result=self.detecting_contours()
                if self.result==True:
                    self.firerow=np.hstack((self.firerow,np.array([[False]])))
                else:
                    self.firerow=np.hstack((self.firerow,np.array([[True]])))
                x0+=self.square_height
            if y0==0:
                self.first_row=self.firerow
            else:
                self.first_row=np.vstack((self.first_row,self.firerow))
            y0 += self.square_width
        self.grid=self.first_row
        self.grid=np.reshape(self.grid,self.division_number*self.division_number)
    def functions(self):
        self.bluring()
        self.converting_to_hsv()
        self.creating_mask()
        self.dilation()
        self.dividing_screen()
    def result(self):
        self.frame=cv.imread(self.path)
        self.functions()
        return self.grid

if __name__=="__main__":
    a=Red_object()
    grid=a.result()
    division=a.division_number
    rospy.wait_for_service(service_name)
    client=rospy.ServiceProxy(service_name, FireData)
    client.call(division,grid)













