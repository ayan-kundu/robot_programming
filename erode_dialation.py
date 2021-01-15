
import numpy ,cv2
#morphological operator : opening (erosion followed by dialation) 
#and closing(dialation followed by erosion)


        
        #namedWindow(" eadge")
		#gray=cv2.cvt_COLOR(cv_image,cv2.COLOR_BGR2GRAY)
		#eadge=cv2.canny(gray,100,200)
		#cv2.imshow("eadge",eadge)


kernel=np.ones((5,5),np.uint8)
opening=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
closing=cv2.morphEx(mask,cv2.MORPH_CLOSE,kernel)

cv2.imshow("Opening",opening)
cv2.imshow("Closing",closing)

k=cv2.waitKey(1000)
if k ==1 :
    break
cv2.destroyAllWindows()

