#Imports
import cv2
import numpy as np
import PositionFinder
import time
import RPi.GPIO as GPIO
from pylepton import Lepton

#Optional Debug Printing
debug = True

#Thresholds
sumThreshold = 60000

#Raspberry Pi Pin declarations
#LEDs
greenLEDPin = 11
yellowLEDPin = 7
redLEDPin = 13
leftHandPin = 37
rightHandPin = 35

#Device Control Pins
makeSafePin = 38
dropGuardPin = 40

#Arduino Communication Pin
commsPin = 36

#Rasperry Pi Pin Booleans
greenLEDPinBool = False
yellowLEDPinBool = False
redLEDPinBool = False
leftHandPinBool = False
rightHandPinBool = False
makeSafePinBool = False
dropGuardPinBool = False

#Configuring GPIO to use board pin numbers, not GPIO numbers
GPIO.setmode(GPIO.BOARD)

def main():

    #Not sure why I need this line
    global redLEDPin

    GPIO.setup(yellowLEDPin,GPIO.OUT)
	GPIO.setup(redLEDPin, GPIO.OUT)
	GPIO.setup(greenLEDPin,GPIO.OUT)
	GPIO.setup(leftHandPin, GPIO.OUT)
	GPIO.setup(rightHandPin, GPIO.OUT)
	GPIO.setup(makeSafePin, GPIO.OUT)
	GPIO.setup(dropGuardPin, GPIO.OUT)
	GPIO.setup(commsPin, GPIO.IN)

	GPIO.output(redLEDPin, False)
	GPIO.output(yellowLEDPin, False)
	GPIO.output(greenLEDPin, False)
	GPIO.output(leftHandPin, False)
	GPIO.output(rightHandPin, False)
	GPIO.output(makeSafePin, False)
	GPIO.output(dropGuardPin, False)

    with Lepton() as camera:
        #Capturing Image
        image,_ = camera.capture()

        #Normilizing Image Pixels
        cv2.normalize(image, image, 0, 65535, cv2.NORM_MINMAX)

        #Fitting data into 8 bits
        np.right_shift(image, 8, image)


        #Initializing PositionFinder
        center = PositionFinder.Position(0,0)

        #Initializing time keeping variables
        timeInitial= = 0
        timeFinal = 0

        #Initializing array that will be used in thresholding
        kernel = np.ones((5,5), np.uint8)

        #Initializing scale factors to improve performance
        scaleFactorOverall = 2 #Used initially after capture
        scaleFactor = .4 #Used only to shrink the difference Image

        #Initializing frame counting variables for FPS Counter
        frameCounter = 0
        frameTime = 0

        #Taking a test frame
        testFrame = cv2.resize(np.uint8(image), None, fx=scaleFactorOverall, fy=scaleFactorOverall)

        #Getting dimentions of video
        width = len(testFrame[0])
        height = len(testFrame)

        #Intializing danger rectangle sizes
        rectHeight = height/5
        rectWidth = width/55

        #Intializing frame array for difference checker
        frameArray = []
        frameArray.append(testFrame)

        #Initializing previousId
        previousId = 0


        #Main program loop
        while True:

            if checkPin(commsPin) == True && activatedMagnet == False:

                #Capturing image, and unique frame id
                img, frame_id = camera.capture()

                #If the frame is a duplicate, try capturing image again
                if frame_id == previousId:
                    if debug:
                        print("Identical")
                    continue

                #Performing formating operations on image to prepare it
                cv2.normalize(img, img, 0, 65535, cv2.NORM_MINMAX)
                np.right_shift(img, 8, img)
                img = cv2.resize(img, None, fx = scaleFactorOverall, fy = scaleFactorOverall)

                img = np.unt8(img)

                #Re-initializing variables (Not sure why)
                greenLEDPinBool = True
				redLEDPinBool = False
				yellowLEDPinBool = False
				leftHandPinBool = False
				rightHandPinBool = False
				makeSafePinBool = False
				dropGuardPinBool = False

                #Initializing/reseting booleans that change every iteration
                inRect = False #If the hands are within danger rectangle
                dangerDifference = False
                dangerVelocity = False

                #Frame Rate Counter
                if frameTime >= 1:
                    if debug:
                        print(frameCounter)
                    frameCounter = 0
                    frameTime = 0

#----------------------------------------------------------------
#-----------------Difference (Danger Check #1)-------------------
#----------------------------------------------------------------
                #Adding current Frame to the frameArray (which now holds the current and previous frames)
                frameArray.append(img)

                #Finding the difference between the two frames to look for movement
                diff = cv2.absdiff(frameArray[0], frameArray[1])

                #Downscaling the difference image for performance when looping
                diffSmall = cv2.resize(diff, None, fx = scaleFactor, fy = scaleFactor)

                #Initializing sum variable
                sum = 0

                #Adding up white value in each pixel (shows movement)
                for pixel in np.nditer(diffSmall, order='c'):
                    sum += pixel

                if sum > sumThreshold:
                    dangerDifference = True
                    if debug:
                        print("Danger Difference: " + str(sum))

#----------------------------------------------------------------
#----------Velocity/Danger Area (Danger Check #2)----------------
#----------------------------------------------------------------
                #Obtaining Average Pixel Value for custom thresholding
                averagePixelValue = np.mean(img)

                #Printing average value if debug is on
                if debug:
                    print("Average Pixel Value: " + str(averagePixelValue))

                #Thresholding image to get pixel values above certain value
                _, threshold1 = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)

                #Refining the threshold by eroding and dilating image
                threshold1_refined = cv2.morphologyEx(threshold1, cv2.MORPH_OPEN, kernal)


                #Finding contours
                contours, hierarchy = cv2.findContours(threshold1_refined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

				for i in range(len(contours)):
					hull.append(cv2.convexHull(contours[i], False))

				if len(contours) == 1:
					leftHandPinBool = True
				elif len(contours) >= 2:
					leftHandPinBool = True
					rightHandPinBool = True

                largestTwoContours = [0,0]
                largestTwoContoursIndexes = [0,0]

                areaArray = []

                for i in range(len(contours)):
                    area = cv2.contourArea(contours[i])
                    areaArray.append([i,area])

				for i in range(len(contours)):
					#Drawing Contours
					area = cv2.contourArea(contours[i])

                    for j in range(len(largestTwoContours)):
                        if area > largestTwoContours[j]:
                            largestTwoContours[j] = area
                            largestTwoContoursIndexes[j] = i
                            break

					if area < 500:
						continue
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




                #Calculating image moments
                M = cv2.moments(threshold1_refined)

                #Calculating Center of mass of hands
                if(M['m00'] != 0):
					centerx = int(M['m10']/M['m00'])
					centery = int(M['m01']/M['m00'])
                #Else statement to assure that variables get assigned
				else:
					centerx = 0
					centery = 0

                #Calculating the change in time since the last frame
                timeFinal = time.time()
                dt = timeFinal-timeInitial

                #Updateing future position prediction
                center.update(centerx, centery, dt)

                #Based on how fast the users hands are moving, the danger rectangle adjusts its sizes
                extraSizeToRecty = int(center.vy)
                if extraSizeToRecty < 0:
                    extraSizeToRecty = 0
                extraSizeToRectx = int(abs(centery.vx**2))/5

                futurePos = center.futurePosition(.2)

                #Drawing stuff on screen
                cv2.circle(img, (centerx, centery), 10, (0,0,255))
				cv2.circle(img, (futurePos), 10, (0,0,255))



                #loop
                #if danger
                    #activatedMagnet = True
            else:
                #waitUntilPin(commsPin, 1, .1)
                activateMagnet = False
