#!/usr/bin/env python
# coding: utf-8

# In[ ]:


def detectcircle(img):
    '''input string--name of the image
       output 1. center of the circle coordinate (array) 2.the RGB value of the center pixel
       if detect no circle, it return 0
    '''
    import cv2
    import numpy as np
    import matplotlib.pyplot as plt
    from PIL import Image
    #%matplotlib inline
    #img=cv2.imread(image_path,0)
    img = cv2.medianBlur(img,5)
    ret,thresh=cv2.threshold(img,127,255,0)
    edges=cv2.Canny(img,150,200)
    cimg = cv2.cvtColor(edges,cv2.COLOR_GRAY2BGR)
    x = y = RGB = 0
    #plt.imshow(edges)
    #plt.gray()
    #plt.show()
    ##############################
    if img is not None:
        circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,param1=60,param2=35,minRadius=0,maxRadius=0)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
        coord = np.where(np.all(cimg == (0, 0, 255), axis=-1))
        #print(coord[1][1])
        x=int(coord[0][1])
        y=int(coord[1][1])

        #im = Image.open(image_path) # Can be many different formats.
        pix = img.load()
        RGB=pix[x,y]
        #print (RGB)  # Get the RGBA Value of the a pixel of an image

        #cv2.imshow('detected circles',cimg)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
    else:
        x = y = RGB = 0
    return [x,y],RGB
if __name__ == "__main__":
   # vid = cv2.VideoCapture(0)
    #while (1):
    path = 'img_1.jpg'
     #   success, img = vid.read()
       # cv2.imshow('img1',test)
       # cv2.waitkey()
    [x,y], RGB = detectcircle(path)
    print ("x,y,RGB", [x,y], RGB)

