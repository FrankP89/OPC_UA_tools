#!/usr/bin/env python2  
#
#import roslib
import rospy
#import math
import tf2_ros
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

	root_base = client.get_root_node()

	opc_zedx = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDx"])
	opc_zedy = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDy"])
	opc_zedz = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDz"])
	opc_zedq1 = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDq1"])
	opc_zedq2 = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDq2"])
	opc_zedq3 = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDq3"])
	opc_zedq4 = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDq4"])
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

	try:
		if is_connected:

			# Initialize node
			rospy.init_node('zed_tf_bridge')

			tfBuffer = tf2_ros.Buffer()
			listener = tf2_ros.TransformListener(tfBuffer)
			
			# Define the rate of spinning
			rate = rospy.Rate(10.0)
			# while True:
			while not rospy.is_shutdown():

				try:
					#listener = tf.TransformListener()
					# Obtain values for the translation and rotation
					#listener.waitForTransform('/aruco_zed_frame', '/object_1', rospy.Time(0), rospy.Duration(4.0))
					#(trans,rot) = listener.lookupTransform('/aruco_zed_frame', '/object_1', rospy.Time.now())

					
					trans = tfBuffer.lookup_transform('aruco_zed_frame', 'object_1', rospy.Time.now(), rospy.Duration(4.0))

					try: 
						# Convert trans and rot information - Extract them as individual values
						ZEDx = trans.transform.translation.x
						ZEDy = trans.transform.translation.y
						ZEDz = trans.transform.translation.z
						ZEDq1 = trans.transform.rotation.x
						ZEDq2 = trans.transform.rotation.y
						ZEDq3 = trans.transform.rotation.z
						ZEDq4 = trans.transform.rotation.w

						#ZEDx = 1.0
						#ZEDy = 2.0
						#ZEDz = 3.0
						#ZEDq1 = 4.0
						#ZEDq2 = 5.0
						#ZEDq3 = 6.0
						#ZEDq4 = 7.0
						
						# Update the UA Address Space
						ua_info_update()

					except:
						print("Failed to write variables")
						rate.sleep()
						continue


				except Exception as e:
					print("Failed to find frame", e)
					time.sleep(2.0)
					continue

			
				rate.sleep()
	finally:
		print("Closing connection")
		client.close_session()
