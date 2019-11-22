import os
import argparse as argp
import cv2
import numpy as np
from Adafruit_BNO055 import BNO055
from time import sleep
from timeit import default_timer as timer

def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('true', 't'):
        return True
    elif v.lower() in ('false', 'f'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def init_imu():
	"""Initializes the BNO055 Adafruit IMU sensor and returns the object"""
	bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
	while True:
		try:
			if not bno.begin():
				raise RuntimeError('Failed to initialize sensor.')
			break;
		except RuntimeError:
			sleep(0.1)

	status, self_test, error = bno.get_system_status()
	print('System status {}'.format(status))
	if status == 0x01:
		print('System error: {}'.format(error))
	return bno

def init_camera(size=(1280,720)):
	"""Initializes camera component and the video writer for saving the video file"""
	cap = cv2.VideoCapture(0)

	cap.set(3,1280)
	cap.set(4,720)

	ret, frame = cap.read()
	output_video = cv2.VideoWriter("output.avi", cv2.VideoWriter_fourcc(*'MJPG'), 27.4, np.shape(frame)[0:2][::-1])
	return cap, output_video

def rotate_image(image, angle):
	"""Rotates image around its center with angle degrees"""
	image_center = tuple(np.array(image.shape[1::-1]) / 2)
	rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.3)
	result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
	return result

def get_euler_roll(bno):
	while True:
		try:
			roll = bno.read_euler()[1]
			return roll
		except RuntimeError:
			pass

def save_recording(seconds, num_frames=27.23):
	"""Records for a given amount of seconds, then saves the recording to disk as 'output.avi'"""
	bno = init_imu()
	cap, output_video = init_camera()

	frame_vector = [np.zeros((720, 1280, 3), dtype=np.uint8)]*(int(seconds*num_frames))

	print("Started...")
	for i in range(int(seconds*num_frames)):
		ret, frame = cap.read()
		curr_roll = bno.read_euler()[1]
		frame_vector[i] = rotate_image(frame, curr_roll)
	print("Finished recording...")

	for index in range(len(frame_vector)):
		output_video.write(frame_vector[index])

	cap.release()
	output_video.release()

def display_recording():
	"""Display the recording in a separate window, doesn't save anything to disk."""
	bno = init_imu()
	cap = init_camera()[0]

	while True:
		ret, frame = cap.read()
		roll_vector = bno.read_euler()[1]
		frame = rotate_image(frame, roll_vector)
		cv2.imshow('Camera 0', frame)
		if cv2.waitKey(1) == 27:
			cap.release()
			cv2.destroyAllWindows();
			break

if __name__ == "__main__":
	# Make process realtime
	niceness = os.nice(0)
	os.nice(-20-niceness)
	print("Nice level of {} selected".format(os.nice(0)))

	max_pri = os.sched_get_priority_max(os.SCHED_RR)
	sched_param = os.sched_param(max_pri)
	os.sched_setscheduler(0, os.SCHED_RR, sched_param)

	# Parser for command-line arguments
	parser = argp.ArgumentParser(description="Take a video and rotate it along the roll Euler vector.")
	parser.add_argument('--seconds', metavar='s', type=int,
		default=5, help="How many seconds to record")
	parser.add_argument('--live', metavar='l', type=str2bool, nargs='?', const=True,
		default=False, help="Should the recording be displayed as a window?")
	args = parser.parse_args()

	start = timer()
	if args.live:
		display_recording()
	else:
		save_recording(args.seconds)
	end = timer()
	print("Total processing time: {}".format(end-start))
