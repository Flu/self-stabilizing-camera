import os
import argparse as argp
import cv2
import numpy as np
from Adafruit_BNO055 import BNO055
from time import sleep

def init_imu():
	# Make process almost realtime
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
	return bno

def init_camera(size=(1280,720)):
	cap = cv2.VideoCapture(0)

	cap.set(3,1280)
	cap.set(4,720)

	ret, frame = cap.read()
	output_video = cv2.VideoWriter("output.avi", cv2.VideoWriter_fourcc(*"XVID"), 27.4, np.shape(frame)[0:2][::-1])
	return cap, output_video

def rotate_image(image, angle):
	"""Rotates image around its center with angle degrees"""
	image_center = tuple(np.array(image.shape[1::-1]) / 2)
	rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.4)
	result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
	return result

def run(seconds):
	bno = init_imu()
	cap, output_video = init_camera()

	frame_vector = list()
	roll_vector = list()

	print("Started...")
	for i in range(int(seconds*30)):
		ret, frame = cap.read()
		frame_vector.append(frame)
		roll_vector.append(bno.read_euler()[1])
	print("Finished recording...")

	for frame, roll in zip(frame_vector, roll_vector):
		output_video.write(rotate_image(frame, roll))

	cap.release()
	output_video.release()

if __name__ == "__main__":
	parser = argp.ArgumentParser(description="Take a video and rotate it along the roll Euler vector.")
	parser.add_argument('seconds', metavar='s', type=int,
		default=5, help="how many seconds to record")
	args = parser.parse_args()
	run(args.seconds)
