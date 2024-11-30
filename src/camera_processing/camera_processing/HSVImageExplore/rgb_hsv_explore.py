# shows the rgb and hsvchannels of an image
# parts from
# https://botforge.wordpress.com/2016/07/02/basic-color-tracker-using-opencv-python/

import cv2
import numpy as np
import sys
import os.path

print("Using python version {0}".format(sys.version))
print('OpenCV Version = ', cv2.__version__)
print()

print(sys.argv)

if 'idlelib' in sys.modules:
    print('Running from IDLE')
    sys.argv = ["rgb_hsv_explore.py", "IMG_test.jpg", "half"]
else:
    print('Running from Python or VSCode')
    if len(sys.argv) != 3:
        sys.argv = ["rgb_hsv_explore.py", "IMG_test.jpg", "half"]

# for idle
# if sys.modules['idlelib']:
#    sys.argv = ["rgb_hsv_explore.py", "IMG_1204.jpg", "half"]
    
#optional argument
def nothing(x):
    pass

if len(sys.argv) != 3:
    print ("%s input_file flip/none/half/quar" % (sys.argv[0]))
    sys.exit()
else:
    input_file = sys.argv[1]
    argFlip = sys.argv[2]

if not os.path.isfile(input_file):
    print ("No such file '%s'" % input_file)
    sys.exit()

#cap = cv2.VideoCapture(0)
# cv2.namedWindow('step1')
cv2.namedWindow("step1", cv2.WINDOW_NORMAL) 
cv2.setWindowProperty('step1', 0, 1)

#easy assigments
hh1='Hue High'
hl1='Hue Low'
sh1='Saturation High'
sl1='Saturation Low'
vh1='Value High'
vl1='Value Low'

cv2.createTrackbar(hl1, 'step1',0,179,nothing) 
cv2.createTrackbar(sl1, 'step1',120,255,nothing) 
cv2.createTrackbar(vl1, 'step1',60,255,nothing) 

cv2.createTrackbar(hh1, 'step1',30,179,nothing) 
cv2.createTrackbar(sh1, 'step1',255,255,nothing)
cv2.createTrackbar(vh1, 'step1',240,255,nothing)

imgOriginal = cv2.imread(input_file)

if argFlip == 'half':
    imgProcess = cv2.resize(imgOriginal, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_LINEAR)
elif argFlip == 'quar':
    imgProcess = cv2.resize(imgOriginal, None, fx=0.25, fy=0.25, interpolation = cv2.INTER_LINEAR)
elif argFlip == 'eighth':
    imgProcess = cv2.resize(imgOriginal, None, fx=0.125, fy=0.125, interpolation = cv2.INTER_LINEAR)
elif argFlip == 'flip':
    imgProcess = cv2.flip(imgOriginal, 0)
elif argFlip == 'full':
    imgProcess = imgOriginal.copy()
else:
    imgProcess = imgOriginal.copy()


#Split out each channel
#imgProcBlue, imgProcGreen, imgProcRed = cv2.split(imgProcess)
imgProcBlue = imgProcess[:,:,0]
imgProcGreen = imgProcess[:,:,1]
imgProcRed = imgProcess[:,:,2]
imgProc_disp = np.hstack([imgProcBlue, imgProcGreen, imgProcRed])

while(1):
#    _,frame=cap.read()

    #read trackbar positions for all
    hul=cv2.getTrackbarPos(hl1, 'step1')
    huh=cv2.getTrackbarPos(hh1, 'step1')
    sal=cv2.getTrackbarPos(sl1, 'step1')
    sah=cv2.getTrackbarPos(sh1, 'step1')
    val=cv2.getTrackbarPos(vl1, 'step1')
    vah=cv2.getTrackbarPos(vh1, 'step1')

    #make array for final values
    hsvlow1 = np.array([hul, sal, val])
    hsvhigh1 = np.array([huh, sah, vah])

    #convert to HSV from BGR, and make gray for display
    imgHsv1 = cv2.cvtColor(imgProcess, cv2.COLOR_BGR2HSV)

    #Split out each channel
    #imgProcHue, imgProcSat, imgProcVal = cv2.split(imgHsv1)
    imgProcHue = imgHsv1[:,:,0]
    imgProcSat = imgHsv1[:,:,1]
    imgProcVal = imgHsv1[:,:,2]
    imgHSV1_disp = np.hstack([imgProcHue, imgProcSat, imgProcVal])

#    cv2.imshow('imgHSV_disp', imgHSV_disp)

    #apply the range on a mask
    imgMask1 = cv2.inRange(imgHsv1, hsvlow1, hsvhigh1)

    imgBitwise1 = cv2.bitwise_and(imgProcess, imgProcess, mask=imgMask1)

    #Split out each channel
    #imgBitBlue, imgBitGreen, imgBitRed = cv2.split(imgBitwise1)
    imgBitBlue = imgBitwise1[:,:,0]
    imgBitGreen = imgBitwise1[:,:,1]
    imgBitRed = imgBitwise1[:,:,2]

##
    # this is step 2 of 4
    shiftlow2 = 100 # was 80
    shifthigh2 = 255   
    
    imgBitwise2b = imgBitwise1.copy()
    imgBitwise2g = imgBitwise1.copy()
    imgBitwise2r = imgBitwise1.copy()

    imgBitwise2b[imgBitwise2b[:, :, 0] > shiftlow2, 0] = shifthigh2 # shift blue
    imgBitwise2g[imgBitwise2g[:, :, 1] > shiftlow2, 1] = shifthigh2 # shift green
    imgBitwise2r[imgBitwise2r[:, :, 2] > shiftlow2, 2] = shifthigh2 # shift red
    
    imgBitBlue = imgBitwise2b[:,:,0]
    imgBitGreen = imgBitwise2g[:,:,1]
    imgBitRed = imgBitwise2r[:,:,2]

    #cv2.imshow('imgBitBlue', imgBitBlue)    
    #cv2.imshow('imgBitGreen', imgBitGreen)    
    #cv2.imshow('imgBitRed', imgBitRed)    

    #while (True):
        #    kp = cv2.waitKey(0) & 0xFF
        #    if kp is 27:
        #        break

    imgEqualBlue = cv2.equalizeHist(imgBitBlue)
    imgEqualGreen = cv2.equalizeHist(imgBitGreen)
    imgEqualRed = cv2.equalizeHist(imgBitRed)

    #cv2.imshow('imgEqualBlue', imgEqualBlue)    
    #cv2.imshow('imgEqualGreen', imgEqualGreen)    
    #cv2.imshow('imgEqualRed', imgEqualRed)    

    #while (True):
        #    kp = cv2.waitKey(0) & 0xFF
        #    if kp is 27:
        #        break
    
    imgEqualYellow = cv2.bitwise_or(imgEqualGreen,imgEqualRed)

##
    imgBitwise2_disp = np.hstack([imgEqualBlue, imgEqualGreen, imgEqualRed])

#    cv2.imshow('imgBitwise1', imgBitwise1_disp)

    imgMerge1a_disp = np.hstack([imgProcess, imgBitwise1])
    imgMerge1b_disp = np.vstack([imgBitwise2_disp, imgHSV1_disp])

    cv2.imshow('step1', imgMerge1a_disp)
    cv2.imshow('channels', imgMerge1b_disp)

    kp = cv2.waitKey(5) & 0xFF
    if kp == 27:
        break

cv2.destroyAllWindows()

# trackbar as step 2
cv2.namedWindow('step2')

#easy assigments
sl2 = 'Saturations >'
sh2 = 'Set to this'

cv2.createTrackbar(sl2, 'step2', 100, 255,nothing)
cv2.createTrackbar(sh2, 'step2', 255, 255,nothing)

while(1):
    #read trackbar positions for all
    sal=cv2.getTrackbarPos(sl2, 'step2')
    sah=cv2.getTrackbarPos(sh2, 'step2')

    imgBitwise2 = imgBitwise1.copy()
    imgBitwise2[imgBitwise2[:, :, 1] > sal, 1] = sah
    #imgBitmore[imgBitmore[:, :, 1] < sal + 2, 1] = 255 - sah

    #Split out each channel
    #imgBitBlue, imgBitGreen, imgBitRed = cv2.split(imgBitmore)
    imgBitBlue = imgBitwise2[:,:,0]
    imgBitGreen = imgBitwise2[:,:,1]
    imgBitRed = imgBitwise2[:,:,2]

    imgNewGreen = cv2.equalizeHist(imgBitGreen)

    cv2.imshow('step2', imgNewGreen)

    kp = cv2.waitKey(5) & 0xFF
    if kp == 27:
        break

cv2.destroyAllWindows()


# trackbar as step 3
# create trackbar for threshold
cv2.namedWindow('step3')

#easy assigments
th3='Threshold'
gs3='Grayscale'

cv2.createTrackbar(th3, 'step3',175,255,nothing) 
cv2.createTrackbar(gs3, 'step3',255,255,nothing)

while(1):
    #read trackbar positions for all
    thv=cv2.getTrackbarPos(th3, 'step3')
    grv=cv2.getTrackbarPos(gs3, 'step3')

    ret, imgThresh = cv2.threshold(imgNewGreen, thv, grv, cv2.THRESH_BINARY)

    cv2.imshow('step3', imgThresh)
    
    imgShowGreen_disp = np.hstack((imgProcGreen, imgBitGreen, imgNewGreen, imgThresh)) 

    imgBitwise3 = cv2.bitwise_and(imgProcess, imgProcess, mask=imgThresh)

    imgMerge3_disp = np.hstack([imgProcess, imgBitwise1, imgBitwise3])

    cv2.imshow('step3 - Original, First mask, Modifications to green -> new mask', imgMerge3_disp)

    kp = cv2.waitKey(5) & 0xFF
    if kp == 27:
        break

cv2.destroyAllWindows()

cv2.namedWindow('step4')

#easy assigments
hh4='Hue High'
hl4='Hue Low'
sh4='Saturation High'
sl4='Saturation Low'
vh4='Value High'
vl4='Value Low'

cv2.createTrackbar(hl4, 'step4',15,179,nothing)
cv2.createTrackbar(hh4, 'step4',100,179,nothing)
cv2.createTrackbar(sl4, 'step4',130,255,nothing) # was 80
cv2.createTrackbar(sh4, 'step4',255,255,nothing)
cv2.createTrackbar(vl4, 'step4',100,255,nothing) # was 50, 175
cv2.createTrackbar(vh4, 'step4',240,255,nothing)

while(1):

    imgHsv4 = cv2.cvtColor(imgBitwise3, cv2.COLOR_BGR2HSV)

    #read trackbar positions for all
    hul=cv2.getTrackbarPos(hl4, 'step4')
    huh=cv2.getTrackbarPos(hh4, 'step4')
    sal=cv2.getTrackbarPos(sl4, 'step4')
    sah=cv2.getTrackbarPos(sh4, 'step4')
    val=cv2.getTrackbarPos(vl4, 'step4')
    vah=cv2.getTrackbarPos(vh4, 'step4')

    #make array for final values
    hsvlow4 = np.array([hul,sal,val])
    hsvhigh4 = np.array([huh,sah,vah])

    #apply the range on a mask
    imgMask4 = cv2.inRange(imgHsv4, hsvlow4, hsvhigh4)

    imgMask4Height,imgMask4Width = imgMask4.shape[:2]

    imgBitwise4 = cv2.bitwise_and(imgProcess,imgProcess, mask=imgMask4)

    imgBlank_disp = np.zeros((imgMask4Height,imgMask4Width), np.uint8)

    imgMerge = cv2.merge((imgBlank_disp,imgMask4,imgBlank_disp))

#    print imgMask4.shape
#    print imgBlank_disp.shape
#    print '-x-'
#    print imgBitwise4.shape
#    print imgMerge.shape

    imgDisplay = np.hstack([imgBitwise4,imgMerge])

#    print imgDisplay.shape

    cv2.imshow('step4', imgDisplay)

    kp = cv2.waitKey(5) & 0xFF
    if kp == ord('s'): # wait for 's' key to save and exit
        write_file = input_file[:len(input_file) - 4]
        cv2.imwrite(write_file + "-result.jpg", imgBitwise4)
        cv2.imwrite(write_file + "-mask.jpg", imgMask4)
        break
    if kp == 27:
        break

cv2.destroyAllWindows()
