import serial

serial0 = serial.Serial("/dev/ttyACM0", 115200, timeout = 0)
serial1 = serial.Serial("/dev/ttyACM1", 115200, timeout = 0)
serial2 = serial.Serial("/dev/ttyACM2", 115200, timeout = 0)'
serial_out = sserial.Serial("/dev/ttyACM3", 115200, timeout = 0)'

# key for distance input
key = "Distance"

#distance variables
d0 = 0
d1 = 0
d2 = 0

# initialize sensor location
front_sensor = (0.0, 20)
left_sensor = (-10, 20)
right_sensor = (10, -20)

# initialize coordinates
sys = system(front_sensor, left_sensor, right_sensor)

print("starting measurements")
while True:
	# read serial line (in byte form) and decode into string
	response0 = (serial0.readline()).decode()
	response1 = (serial1.readline()).decode()
	response2 = (serial2.readline()).decode()
	
	# parse input for distance
	if key in response0:
		response0 = response0.split(" : ")
		# check for proper list length, previously gave an index out 
		# of range error
		if len(response0) == 2:
			d0 = float(response0[1])*100
	if key in response1:
		response1 = response1.split(" : ")
		if len(response1) == 2:
			d1 = float(response1[1])*100
	if key in response2:
		response2 = response2.split(" : ")
		if len(response2) == 2:
			d2 = float(response2[1])*100
	'''
	# print distance to tag, for debugging purposes
	print(d0)
	print(d1)
	print(d2)
	'''
	
	# coordinate system
	coordinate = sys.trilaterate(d0, d1, d2)
	
	# ignore z-axis 
	coord_final = coordinate[0], coordinate[1]
	'''
	# print coordinate, for debugging purposes
	print(coord_final)
	'''
	
	# send coordinates to arduino
	'''
	NOTE: port ttyACM3 will output the tuple of coordinates in byte form, decode upon reception.
	'''
	serial_out.write(coord_final.encode()))
