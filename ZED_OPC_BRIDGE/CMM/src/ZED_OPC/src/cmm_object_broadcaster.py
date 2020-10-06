#!/usr/bin/env python

'''
	October 6th, 2020. 
	ARTC - ARA - CRP7 - 5G
	cmm_object_broadcaster.py
	This program will serve as a frame broadcaster after
	receiving the information from the OPC UA address space.
	Developers:
	* Lim Guo Wei
	* Walter Pintor
'''

import rospy
import tf

from opcua import Client
from opcua import ua
''' 
	This client will receive the information broadcasted.
	It will then pass the info to ROS.
'''

# Change IP where needed
url = "opc.tcp://127.0.0.1:4840/freeopcua/server/"
opc_client = Client(url)

def opc_client_connect():
	# OPC-UA connection
	try:
		opc_client.connect()
		print("OPC Client Connected")
		return True
	except:
		print("OPC Client NOT Connected")
		return False

def close():	
	try:
		print("Attempting to close...")
		opc_client.disconnect()	
		print("Closed connection with OPC-UA Server")
	except: 
		pass
		print("Failed to close")

# Define variables
tx = 0.0
ty = 0.0
tz = 0.0
qx = 0.0
qy = 0.0
qz = 0.0
qw = 1.0

def get_object_frame_from_opc():
	global tx
	global ty
	global tz
	global qx
	global qy
	global qz
	global qw

	root_base = opc_client.get_root_node()

	opc_zedx = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDx"])
	opc_zedy = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDy"])
	opc_zedz = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDz"])
	opc_zedq1 = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDq1"])
	opc_zedq2 = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDq2"])
	opc_zedq3 = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDq3"])
	opc_zedq4 = root_base.get_child(["0:Objects", "2:Variables", "2:fZEDq4"])
	print("Values updated from address space")
	
	tx = opc_zedx.get_value()
	ty = opc_zedy.get_value()
	tz = opc_zedz.get_value()
	qx = opc_zedq1.get_value()
	qy = opc_zedq2.get_value()
	qz = opc_zedq3.get_value()
	qw = opc_zedq4.get_value()

	trans = (tx, ty, tz)
	rot = (qx, qy, qz, qw)
	return trans, rot 

def object_broadcaster(trans, rot):
	br = tf.TransformBroadcaster()
	br.sendTransform(trans, rot, rospy.Time.now(), objectName, "/aruco_marker_frame")

	return
	
if __name__ == "__main__":

	# Connect to OPC-UA server	
	is_connected = opc_client_connect()

	try:
		if is_connected:
			rospy.init_node("object_broadcaster")
			
			rospy.set_param("~stationName", "cmm")	# To be removed and set from launch file instead
			stationName = rospy.get_param("~stationName")

			rospy.set_param("~objectName", "object_1")	# To be removed and set from launch file instead
			objectName = rospy.get_param("~objectName")
			
			rate = rospy.Rate(10) # 10 hz
			while not rospy.is_shutdown():
			
				trans, rot = get_object_frame_from_opc()
				object_broadcaster(trans, rot)	
				rate.sleep()

			rospy.spin()

	finally:
		print("Closing connection")
		client.close_session()
