import cv2
import numpy as np
def main():
    import sys
    image_src = cv2.imread('simulation.png')  # pick.py my.png
    if image_src is None:
        print ("the image read is None............")
        return
    cv2.imshow("bgr",image_src)
    img_color_hsv = cv2.cvtColor(image_src,cv2.COLOR_BGR2HSV)
    # lower = np.array([ 0, 120, 140])    # hsv value of "lighter red"
    lower = np.array([ 4, 198, 0])    # hsv value of "lighter red"
    # upper = np.array([ 15, 167, 207])   # hsv value of "darker red"
    upper = np.array([ 24, 218, 67])   # hsv value of "darker red"
    # image_mask = cv2.inRange(img_color_hsv,hsv_yellow1,hsv_yellow2)
    image_mask = cv2.inRange(img_color_hsv,lower,upper)
    cv2.imshow("mask",image_mask)

    h, w, _ = image_src.shape

    search_top = 3*h/4
    search_bot = search_top + 20
    print "search top = {top}, search bottom = {bot}".format(top=search_top, bot=search_bot)

    # erase all the point in the mask outside the boundaries
    image_mask[0:search_top, 0:w] = 0
    image_mask[search_bot:h, 0:w] = 0

    cv2.imshow("mask_cropped", image_mask)


    #BEGIN FINDER
    M = cv2.moments(image_mask)
    if M['m00']>0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    #End  Finder

    # Begin Circle
    cv2.circle(image_src, (cx,cy), 20, (0,0,255), -1)
    #End Circle

    cv2.imshow("window",image_src)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    

if __name__=='__main__':
   main()
