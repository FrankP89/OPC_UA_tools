#!/usr/bin/env python  
#
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import time

from opcua import Client
from opcua import ua

url = "opc.tcp://127.0.0.1:4840/freeopcua/server/"
client = Client(url)

# Declare variables to update from ROS
ZEDx = 0
ZEDy = 0
ZEDz = 0
ZEDq1 = 0
ZEDq2= 0
ZEDq3 = 0
ZEDq4 = 0

def opc_client_connect():
	# OPC-UA connection
	try:
		client.connect()
		print("OPC Client Connected")
		return True
	except:
		print("OPC Client NOT Connected")
		return False

def close():	
	try:
		print("Attempting to close...")
		client.disconnect()	
		print("Closed connection with OPC-UA Server")
	except: 
		pass
		print("Failed to close")
	


def ua_info_update():

	global ZEDx
	global ZEDy
	global ZEDz
	global ZEDq1
	global ZEDq2
	global ZEDq3
	global ZEDq4


	opc_zedx = client.get_node("ns=2;s=Variables.fZEDx")
	opc_zedy = client.get_node("ns=2;s=Variables.fZEDy")
	opc_zedz = client.get_node("ns=2;s=Variables.fZEDz")
	opc_zedq1 = client.get_node("ns=2;s=Variables.fZEDq1")
	opc_zedq2 = client.get_node("ns=2;s=Variables.fZEDq2")
	opc_zedq3 = client.get_node("ns=2;s=Variables.fZEDq3")
	opc_zedq4 = client.get_node("ns=2;s=Variables.fZEDq4")
	print("Value from UA space: ", opc_zedx)

	dv = ua.DataValue(ua.Variant(ZEDx, ua.VariantType.Float))
	opc_zedx.set_value(dv)
	dv = ua.DataValue(ua.Variant(ZEDy, ua.VariantType.Float))
	opc_zedy.set_value(dv)
	dv = ua.DataValue(ua.Variant(ZEDz, ua.VariantType.Float))
	opc_zedz.set_value(dv)
	dv = ua.DataValue(ua.Variant(ZEDq1, ua.VariantType.Float))
	opc_zedq1.set_value(dv)
	dv = ua.DataValue(ua.Variant(ZEDq2, ua.VariantType.Float))
	opc_zedq2.set_value(dv)
	dv = ua.DataValue(ua.Variant(ZEDq3, ua.VariantType.Float))
	opc_zedq3.set_value(dv)
	dv = ua.DataValue(ua.Variant(ZEDq4, ua.VariantType.Float))
	opc_zedq4.set_value(dv)

	time.sleep(1)


if __name__ == '__main__':
	# Connect to OPC-UA server first	
	is_connected = opc_client_connect()

	if is_connected:

		# Initialize node
		#rospy.init_node('zed_tf_bridge')

		#listener = tf.TransformListener()
		
		# Define the rate of spinning
		#rate = rospy.Rate(10.0)
		#while not rospy.is_shutdown():
		while True:
			# Obtain values for the translation and rotation
			#(trans,rot) = listener.lookupTransform('/aruco_zed_frame', '/object_1', rospy.Time(0))
			#print(type(trans[0]))
			#print(trans)
			try: 
				# Convert trans and rot information - Extract them as individual values
				#ZEDx = trans[0]
				#ZEDy = trans[1]
				#ZEDz = trans[2]
				#ZEDq1 = rot[0]
				#ZEDq2 = rot[1]
				#ZEDq3 = rot[2]
				#ZEDq4 = rot[3]

				ZEDx = 1.0
				ZEDy = 2.0
				ZEDz = 3.0
				ZEDq1 = 4.0
				ZEDq2 = 5.0
				ZEDq3 = 6.0
				ZEDq4 = 7.0
				

				# Update the UA Address Space
				ua_info_update()

			except:
				continue

		#except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			#continue

		
		#rate.sleep()
