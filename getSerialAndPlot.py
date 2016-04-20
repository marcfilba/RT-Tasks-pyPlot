import serial
import select
import numpy as np

from sys import stdin
from matplotlib import pyplot as plt

serialPort = '/dev/ttyACM0'
baudRate = 115200

ser = serial.Serial(serialPort, baudRate)

plt.gcf().set_size_inches(15, 7, forward=True)
plt.ion() # set plot to animated

task1  = [0] * 1000
task2  = [0] * 1000
task3  = [0] * 1000

ax1 = plt.axes()

plt.figure(1).subplots_adjust(hspace = .5)
plt.gcf().set_size_inches(15, 7, forward=True)
plt.subplot(111).set_title("Task Scheduling")

t1, = plt.plot(task1, label = "motor control (20 ms)", drawstyle='steps-pre')
t2, = plt.plot(task2, label = "generate and send local speed (100 ms)", drawstyle='steps-pre')
t3, = plt.plot(task3, label = "receive and average speed (100 ms)", drawstyle='steps-pre')
plt.legend(loc='center left', bbox_to_anchor=(-0.155, 0.5), prop={'size':9})

plt.ylim([-4,4])

rawData = ''
rawDataSplitted = []

while True:

	if select.select([ser,],[],[],0.0)[0]:
		rawData = ser.readline().rstrip()	# llegim accel, gyro i temp
		rawDataSplitted = rawData.split (",")

		if " " in rawData:
			print rawData
	try:
		if len(rawDataSplitted) == 3:
			task1.append(int (str(rawDataSplitted[0]))+2)
			task2.append(int (str(rawDataSplitted[1])))
			task3.append(int (str(rawDataSplitted[2]))-2)

			plt.subplot(111)

			del task1 [0]
			del task2 [0]
			del task3 [0]

			t1.set_xdata(np.arange(len(task1)))
			t1.set_ydata(task1)

			t2.set_xdata(np.arange(len(task2)))
			t2.set_ydata(task2)

			t3.set_xdata(np.arange(len(task3)))
			t3.set_ydata(task3)

			plt.draw()
	except Exception as e:
		pass
