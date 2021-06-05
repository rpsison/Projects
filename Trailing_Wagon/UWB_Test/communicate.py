import serial

# Initialize serial info
serial0 = serial.Serial("/dev/ttyACM0", 115200, timeout = 0)
serial1 = serial.Serial("/dev/ttyACM1", 115200, timeout = 0)
serial2 = serial.Serial("/dev/ttyACM2", 115200, timeout = 0)
serial_out = sserial.Serial("/dev/ttyACM3", 115200, timeout = 0)

# Key for distance input
key = "Distance"

def read_distance():

	#distance variables
	d0 = 0
	d1 = 0
	d2 = 0

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
	return d0, d1, d2
	
def send_coordinate(coordinate):
	serial_out.write(coord_final.encode())