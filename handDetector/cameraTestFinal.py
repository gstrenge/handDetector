import cv2
import numpy as np
import PositionFinder
import time
import RPi.GPIO as GPIO
#import winsound
from threading import Thread
from pylepton import Lepton


slightDangerPin = 7
dropGuardPin = 13
runningPin = 11	

runningPinBool = False
slightDangerPinBool = False
dropGuardPinBool = False


	

def playWarning():
	#winsound.Beep(1000,90)
	print("Drop Guard")
	
	
	

def main():	
		
	
	
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(slightDangerPin,GPIO.OUT)
	GPIO.setup(dropGuardPin,GPIO.OUT)
	GPIO.setup(runningPin,GPIO.OUT)
	
	GPIO.output(dropGuardPin, False)
	GPIO.output(slightDangerPin, False)
	GPIO.output(runningPin, False)
	
	
	with Lepton() as l:
		
		a,_ = l.capture()
		cv2.normalize(a, a, 0, 65535, cv2.NORM_MINMAX) # extend contrast
		np.right_shift(a, 8, a) # fit data into 8 bits
		
		
		
		#cap = cv2.VideoCapture(0)

		center = PositionFinder.Position(0,0)

		timeInitial = 0
		timeFinal = 0

		kernel = np.ones((5,5),np.uint8)



		scaleFactorOverall = 2
		scaleFactor = .4

		frameCounter = 0
		frameTime = 0

		#ret, testFrame = cap.read()
		testFrame = cv2.resize(np.uint8(a), None, fx=scaleFactorOverall, fy=scaleFactorOverall) 
		#testFrame = cv2.resize(testFrame, None, fx = scaleFactorOverall, fy = scaleFactorOverall)

		#testFrame = cv2.resize(testFrame, (0,0), fx=5, fy=5) 

		#Dimensions of Video
		width = len(testFrame[0])
		height = len(testFrame)	
			
		rectHeight = height/5
		print rectHeight
		
		rectWidth = width/55
		
		print rectWidth

		frameArray = []
		frameArray.append(testFrame) #cv2.bilateralFilter(cv2.cvtColor(testFrame, cv2.COLOR_BGR2GRAY),9,75,75)

		#time.sleep(2)
		
		while True:

			#----------------Getting Frame of Video Feed--------------------
			#ret, img = cap.read()
			img, frame_id = l.capture()
			cv2.normalize(img, img, 0, 65535, cv2.NORM_MINMAX) # extend contrast
			np.right_shift(img, 8, img) # fit data into 8 bits
			img = cv2.resize(img, None, fx = scaleFactorOverall, fy = scaleFactorOverall)
			
			img = np.uint8(img)
			
			#img = cv2.resize(img, (0,0), fx=5, fy=5) 
			
			runningPinBool = True
			dropGuardPinBool = False
			slightDangerPinBool = False
			
			#---------------Frame Rate Counter---------------------
			if frameTime >= 1:
				print(frameCounter)
				frameCounter = 0
				frameTime = 0
				
				
			#-----------Intializing Booleans that indicate Danger--------------
			#If hand is in the danger Rectange
			inRect = False
			
			#If both of these danger indicators agree (are True), then the blade guard will drop
			dangerDifference = False
			dangerVelocity = False
			
			
			
			
			#----------------------------------------------------------------
			#-----------------Difference (Danger Check #1)-------------------
			#----------------------------------------------------------------
			#Getting Grayscale Version of Frame
			#gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	
			gray = img
				
			#Blurring Grayscale Image for better processing
			grayBlurred = cv2.bilateralFilter(gray,9,75,75)
			
			
			
			#Adding Blurred Gray Image to difference checker Array
			frameArray.append(grayBlurred)
			
			#Finding the difference between the two frames (Looking for change)
			
			

			diff = cv2.absdiff(frameArray[0], frameArray[1])
			
			#Downscaling for performance
			diffSmall = cv2.resize(diff, None, fx = scaleFactor, fy = scaleFactor)
			
			#Adding upwhite value in each pixel (shows movement)
			sum=0
			for pixel in np.nditer(diffSmall, order='C'):
				sum += pixel
			
			#sum = cv2.sumElems(diffSmall)
			#sum = frame_id
			
			
			print("Sum: " + str(sum))
				
			#If there is enough movement, there is danger (60000 is just a tested value that works)
			if sum > 60000:
				#dangerDifference = True
				slightDangerPinBool = True
				
		
		
			
				
				
				
				
			
			
			
			
			#Thresholding to only bright objects -------------------------------------------To Add Infared Stuff Here
			ret,thresh1 = cv2.threshold(grayBlurred,100,255,cv2.THRESH_BINARY)
			
			#Using Special Threshold Method
			#ret2, thresh2 = cv2.threshold(thresh1, 0,255,cv2.THRESH_BINARY + cv2.THRESH_OTSU)
			
			thresh2 = thresh1
			
			#Erode/Dilate (Look at cv2.MORPH_OPEN/ClOSE on docs to see which goes first)
			thresh3 = cv2.morphologyEx(thresh2, cv2.MORPH_OPEN, kernel)
			
			
			#-------------------Calculating Center of Mass-----------------
			M = cv2.moments(thresh3)
			
			if(M['m00'] != 0):
			
				centerx = int(M['m10']/M['m00'])
				centery = int(M['m01']/M['m00'])
			
			else:
				centerx = 0
				centery = 0
			
			#-----------------Calculating the velocity-------------------
			#Calculating the change in time since the last frame
			timeFinal = time.time()
			dt = timeFinal - timeInitial

			
			#----------------Updating Velocity/Future Position--------------------
			#Updating position/velocity
			center.update(centerx, centery, dt)
			
			#Based on how fast the users hands are moving, the danger rectangle increases/decreases in size
			
			
			extraSizeToRecty = int((center.vy))
			if extraSizeToRecty < 0:
				extraSizeToRecty = 0
			extraSizeToRectx = int(abs(center.vx**2))/5
			
			#Prediciting future position
			futurePos = center.futurePosition(.2)
			
			
			#-----------------Drawing Velocity/Position on Screen-------------------
			#cv2.circle(img, (centerx, centery), 10, (0,0,255))
			#cv2.circle(img, (futurePos), 10, (0,0,255))
			#cv2.arrowedLine(img, (centerx, centery), (futurePos[0], futurePos[1]), (0,0,255), 3)
			
			
			#----------------------------------------------------------------
			#----------Velocity/Danger Area (Danger Check #2)----------------
			#----------------------------------------------------------------
			
			#Finding Contours in black and white image
			contours, hierarchy = cv2.findContours(thresh3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)   #NONE
			
			#Initiating Hull Array
			hull = []
			
			#Filling Convex Hull array with points
			
			for i in range(len(contours)):
				hull.append(cv2.convexHull(contours[i], False))
			
			for i in range(len(contours)):
				#Drawing Contours
				cv2.drawContours(img, contours, i, (255,255,255), 1, 1, hierarchy)
				#Drawing Hull
				#cv2.drawContours(img, hull, i, (255,0,0), 3, 8)
				
				for j in range(len(contours[i])):

					if contours[i][j][0][1] > height - rectHeight - extraSizeToRecty:
						if contours[i][j][0][0] > width/2 - rectWidth - extraSizeToRectx/2 and contours[i][j][0][0] < width/2 + rectWidth + extraSizeToRectx/2 :
							inRect = True

			
			#------------If the hand is in the Danger Rectangle, do action
			if inRect:
				color = (255,255,255)
				dangerVelocity = True
			else:
				color = (255,255,255)
				
			
			#Drawing Danger Rectangle
			cv2.rectangle(img, (width/2-rectWidth - extraSizeToRectx/2 , height - rectHeight - extraSizeToRecty), (width/2+rectWidth + extraSizeToRectx/2 , height), color, 0)
			


			#cv2.imshow("img", img)
			large = cv2.resize(img, (0,0), fx=5, fy=5) 
			cv2.imshow("large", large)
			#cv2.imshow("diff", diffSmall)
			
			#Updating currentFrame to be LastFrame
			frameArray[0] = frameArray[1]
			frameArray.pop(1)
			
			timeInitial = time.time()
			
			if dangerDifference or dangerVelocity:
				#print("Multi")
				dropGuardPinBool = True
				runningPinBool = False
				thread = Thread(target=playWarning)
				thread.daemon = True
				thread.start()
			
			k = cv2.waitKey(45) & 0xff
			if k == 27:
				break
			
			#Adding to Frame Counter
			frameCounter += 1
			frameTime += dt
			
			
			GPIO.output(dropGuardPin, dropGuardPinBool)
			GPIO.output(runningPin, runningPinBool)
			GPIO.output(slightDangerPin, slightDangerPinBool)

			
		#cap.release()
		cv2.destroyAllWindows()


try:
	main()
except Exception as e:
	print e
	
	GPIO.output(dropGuardPin, False)
	GPIO.output(slightDangerPin, False)
	
	sdp = False
	dgp = False
	
	for i in range(5):
		GPIO.output(dropGuardPin, -dgp)
		GPIO.output(slightDangerPin, -sdp)
		time.sleep(1)

finally:
    GPIO.cleanup()
    


