import os
import cv2
import numpy as np
from Adafruit_BNO055 import BNO055
from time import sleep

def rotate_image(image, angle):
	"""Rotates image around its center with angle degrees"""
	image_center = tuple(np.array(image.shape[1::-1]) / 2)
	rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.4)
	result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
	return result

#Make process real-time
niceness = os.nice(0)
os.nice(-20-niceness)

#Create and configure the accelerometer
bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
if not bno.begin():
	raise RuntimeError('Failed to initialize sensor.')

status, self_test, error = bno.get_system_status()
print('System status {}'.format(status))
if status == 0x01:
	print('System error: {}'.format(error))

#Start capturing video
cap = cv2.VideoCapture(0)

cap.set(3,1280)
cap.set(4,720)

sleep(1)
ret, frame = cap.read()
output_video = cv2.VideoWriter("output.avi", cv2.VideoWriter_fourcc(*"XVID"), 27.4, np.shape(frame)[0:2][::-1])

frame_vector = list()
roll_vector = list()

print("Started...")
for i in range(int(5*30)):
	ret, frame = cap.read()
	frame_vector.append(frame)
	roll_vector.append(bno.read_euler()[1])
	# cv2.imwrite("img{0:03d}.jpg".format(i), frame)

print("Finished recording...")

for frame, roll in zip(frame_vector, roll_vector):
	output_video.write(rotate_image(frame, roll))

cap.release()
output_video.release()
