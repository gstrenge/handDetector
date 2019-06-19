# contours = [123,435,232,5454,90034, 10]
# largestTwoContours = [0,0]
# largestTwoContoursIndexes = [0,0]
#
#
#
# for i in range(len(contours)):
#     #Drawing Contours
#     area = contours[i]
#
#     for j in range(len(largestTwoContours)):
#         if area > largestTwoContours[j]:
#             largestTwoContours[j] = area
#             largestTwoContoursIndexes[j] = i
#
#
# print(largestTwoContours)
# print(largestTwoContoursIndexes)


import RPi.GPIO as GPIO
import time
def checkPin(pinNum):
	return GPIO.input(pinNum)


commsPin = 36
GPIO.setmode(GPIO.BOARD)
GPIO.setup(commsPin, GPIO.IN)

while True:
    print(checkPin(36))
    time.sleep(1)
