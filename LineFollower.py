#!/usr/bin/env python3
import ev3dev.ev3 as ev3
from ev3dev2.motor import LargeMotor, MediumMotor
from ev3dev2.sensor.lego import TouchSensor, InfraredSensor
from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
import time
import sys


class LineFollower:
	# Class representing line follower
	def __init__(self) -> None:
		self._left_motor = LargeMotor('outA')
		self._right_motor = LargeMotor('outB')
		self._lifter = MediumMotor('outC')
		self._left_sensor = ColorSensor(address='4')
		self._right_sensor = ColorSensor(address='1')
		self._button = TouchSensor(address='3')
		self._distance_sensor= InfraredSensor(address='2')

		sound = Sound()
		sound.speak('Now!')
		# Settings of the line follower
		self._base_speed = 250
		self._turn_speed = 150
		self._dt = 0.01
		self._kp = 9
		self._Ti = 1.5
		self._Td = 0.08
		self._integral = 0
		self._prev_diff = 0

		self._on_first_line = [0]
		self._on_second_line = 0
		self._has_pacage = 0
		self._turned_left = 1

	def pid_line_follower(self):
		# Method for transporter algorithm
		# Settings of the line follower specific for transporter method:
		self._turn_speed = 175
		self._base_speed = 250
		self._kp = 9
		self._Ti = 1.2
		self._Td = 0.005

		self._right_sensor.mode = 'RGB-RAW'
		self._right_sensor.calibrate_white()
		self._left_sensor.mode = 'RGB-RAW'
		self._left_sensor.calibrate_white()
		while not self._button.is_pressed:
			pass
		while (True):
			first_sens = self._left_sensor.rgb
			second_sens = self._right_sensor.rgb

			difference = (first_sens[0] - second_sens[0])*100/255
			self._integral += self._dt * (self._prev_diff + difference)/2
			deriv = (self._prev_diff - difference)/self._dt
			ster_val = difference * self._kp + self._Ti * self._integral + self._Td * deriv
			self._prev_diff = difference

			if ster_val >= 230:
				# hard turn left
				self._left_motor.run_forever(speed_sp=-self._turn_speed - ster_val/3.5)
				self._right_motor.run_forever(speed_sp= self._turn_speed*2 + ster_val/3.5)
				continue
			elif ster_val <= -230:
				# hard turn right
				self._right_motor.run_forever(speed_sp=-self._turn_speed + ster_val/3.5)
				self._left_motor.run_forever(speed_sp= self._turn_speed*2 - ster_val/3.5)
				continue
			self._right_motor.run_forever(speed_sp=self._base_speed + ster_val/2)
			self._left_motor.run_forever(speed_sp= self._base_speed - ster_val/2)
			# methods responsible for transporting the package
			self.sense_first_turn(first_sens, second_sens)
			self.sense_package()
			self.return_to_line(first_sens, second_sens)
			self.sense_second_turn(first_sens, second_sens)
			self.drop_package(first_sens, second_sens)

	def pid_line_follower_li_intens(self):
		while (True):
			difference = self._left_sensor.reflected_light_intensity - self._right_sensor.reflected_light_intensity
			proportional = self._kp*difference
			self._integral += self._dt * (self._prev_diff + difference)/2
			deriv = (self._prev_diff - difference)/self._dt

			ster_val = proportional + self._Ti * self._integral + self._Td * deriv
			self._prev_diff = difference
			if ster_val >= 70:
				self._left_motor.run_forever( speed_sp=-self._base_speed - ster_val)
				self._right_motor.run_forever(speed_sp= self._base_speed*2 + ster_val)
				continue
			elif ster_val <= -20:
				self._right_motor.run_forever(speed_sp=-self._base_speed + ster_val)
				self._left_motor.run_forever(speed_sp= self._base_speed*2 - ster_val)
				continue
			ster_val = ster_val - 25
			self._right_motor.run_forever(speed_sp=self._base_speed + ster_val)
			self._left_motor.run_forever(speed_sp= self._base_speed - ster_val)

	def sense_first_turn(self, left_sens, right_sens):
		if left_sens[2] >= left_sens[1] + left_sens[0] + 10 and right_sens[2] >= right_sens[1] + right_sens[0] + 10:
			return
		if not self._on_first_line[0] and not self._has_pacage and left_sens[2] >= left_sens[1] + left_sens[0] + 10 and left_sens[2] > 110:
			self._on_first_line[0] = True
			start = time.time()
			while(time.time() - start < 0.35):
				self._left_motor.run_timed(time_sp=4000, speed_sp=self._base_speed*2)
				self._right_motor.run_timed(time_sp=4000, speed_sp=-self._base_speed*2)
			start = time.time()
			self._turned_left = -1
			return
		if not self._on_first_line[0] and not self._has_pacage and right_sens[2] >= right_sens[1] + right_sens[0] + 10 and left_sens[2] > 110:
			self._on_first_line[0] = True
			start = time.time()
			while(time.time() - start < 0.35):
				self._left_motor.run_timed(time_sp=4000, speed_sp = -self._base_speed*2)
				self._right_motor.run_timed(time_sp=4000, speed_sp = self._base_speed*2)
			self._turned_left = 1
			start = time.time()

	def sense_second_turn(self, left_sens, right_sens):
		if not self._on_second_line and self._has_pacage and left_sens[0] >= left_sens[1] + left_sens[2]-10:
			self._on_second_line = True
			start = time.time()
			while(time.time() - start < 0.2):
				pass
			start = time.time()
			while(time.time() - start < 0.575):
				self._left_motor.run_timed(time_sp=4000, speed_sp=self._base_speed*2)
				self._right_motor.run_timed(time_sp=4000, speed_sp=-self._base_speed*2)
			start = time.time()
			while(time.time() - start < 1.2):
				self._left_motor.run_timed(time_sp=4000, speed_sp = self._base_speed)
				self._right_motor.run_timed(time_sp=4000, speed_sp = self._base_speed)
			return
		if not self._on_second_line and self._has_pacage and right_sens[0] >= right_sens[1] + right_sens[2]-10:
			self._on_second_line = True
			start = time.time()
			while(time.time() - start < 0.2):
				pass
			start = time.time()
			while(time.time() - start < 0.575):
				self._left_motor.run_timed(time_sp=4000, speed_sp = -self._base_speed*2)
				self._right_motor.run_timed(time_sp=4000, speed_sp = self._base_speed*2)
			start = time.time()
			while(time.time() - start < 1.2):
				self._left_motor.run_timed(time_sp=4000, speed_sp = self._base_speed)
				self._right_motor.run_timed(time_sp=4000, speed_sp = self._base_speed)

	def sense_package(self):
		if not self._has_pacage and self._distance_sensor.proximity < 30:
			robot._right_motor.stop()
			robot._left_motor.stop()
			start = time.time()
			while time.time() - start < 1:
				 self._lifter.run_timed(time_sp=1000, speed_sp = 200)
			start = time.time()
			while(time.time() - start < 1):
				self._left_motor.run_timed(time_sp=4000, speed_sp = -self._base_speed*2)
				self._right_motor.run_timed(time_sp=4000, speed_sp = self._base_speed*2)
			start = time.time()
			while(time.time() - start < 0.25):
				self._left_motor.run_timed(time_sp=4000, speed_sp = self._base_speed*2)
				self._right_motor.run_timed(time_sp=4000, speed_sp = self._base_speed*2)
			self._has_pacage = True

	def return_to_line(self, left_sens, right_sens):
		if self._on_first_line[0] and self._has_pacage and left_sens[0] + left_sens[1] + left_sens[2] >= 3* 235 and right_sens[0] + right_sens[1] + right_sens[2] >= 3* 235:
			start = time.time()
			while(time.time() - start < 0.6):
				self._left_motor.run_timed(time_sp=4000, speed_sp = -self._base_speed*2 * self._turned_left)
				self._right_motor.run_timed(time_sp=4000, speed_sp = self._base_speed*2 * self._turned_left)
			self._on_first_line[0] = 0

	def drop_package(self, right_sens, left_sens):
		if self._has_pacage and self._on_second_line and right_sens[0] >= right_sens[1] + right_sens[2]-10 and left_sens[0] >= left_sens[1] + left_sens[2]-10:
			self._right_motor.stop()
			self._left_motor.stop()
			start = time.time()
			while(time.time() - start < 0.5):
				self._lifter.run_timed(time_sp=200, speed_sp = -100)
			while(time.time() - start < 0.8):
				self._left_motor.run_timed(time_sp=4000, speed_sp = -self._base_speed)
				self._right_motor.run_timed(time_sp=4000, speed_sp = -self._base_speed)
			sound = Sound()
			sound.speak('Done!')
			sys.exit()

robot = LineFollower()
try:
	robot.pid_line_follower()
except:
	robot._right_motor.stop()
	robot._left_motor.stop()
