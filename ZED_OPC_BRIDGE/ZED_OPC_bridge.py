#!/usr/bin/env python  
import roslib
roslib.load_manifest('zed_tf')
import rospy
import math
import tf
import geometry_msgs.msg

from opcua import Client
from opcua import ua

url = "opc.tcp://127.0.0.1:48050"
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
	except:
		print("OPC Client NOT Connected")
		pass

def close():	
	try:
		print("Attempting to close...")
		client.disconnect()	
		print("Closed connection with OPC-UA Server")
	except: 
		pass
		print("Failed to close")
	


def ua_info_update():
	opc_zedx = client.get_node("ns=6;s=Variables.fZEDx")
	opc_zedy = client.get_node("ns=6;s=Variables.fZEDy")
	opc_zedz = client.get_node("ns=6;s=Variables.fZEDz")
	opc_zedq1 = client.get_node("ns=6;s=Variables.fZEDq1")
	opc_zedq2 = client.get_node("ns=6;s=Variables.fZEDq2")
	opc_zedq3 = client.get_node("ns=6;s=Variables.fZEDq3")
	opc_zedq4 = client.get_node("ns=6;s=Variables.fZEDq4")

	dv = ua.DataValue(ua.Variant(ZEDx, ua.VariantType.Double))
	opc_zedx.set_value(dv)
	dv = ua.DataValue(ua.Variant(ZEDy, ua.VariantType.Double))
	opc_zedy.set_value(dv)
	dv = ua.DataValue(ua.Variant(ZEDz, ua.VariantType.Double))
	opc_zedz.set_value(dv)
	dv = ua.DataValue(ua.Variant(ZEDq1, ua.VariantType.Double))
	opc_zedq1.set_value(dv)
	dv = ua.DataValue(ua.Variant(ZEDq2, ua.VariantType.Double))
	opc_zedq2.set_value(dv)
	dv = ua.DataValue(ua.Variant(ZEDq3, ua.VariantType.Double))
	opc_zedq3.set_value(dv)
	dv = ua.DataValue(ua.Variant(ZEDq4, ua.VariantType.Double))
	opc_zedq4.set_value(dv)


if __name__ == '__main__':
	# Connect to OPC-UA server first	
	opc_client_connect()

	# Initialize node
	rospy.init_node('zed_tf_bridge')

	listener = tf.TransformListener()
	
	# Define the rate of spinning
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			# Obtain values for the translation and rotation
			(trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
			try: 
				# Convert trans and rot information - Extract them as individual values
				ZEDx = trans[0]
				ZEDy = trans[1]
				ZEDz = trans[2]
				ZEDq1 = rot[0]
				ZEDq2 = rot[1]
				ZEDq3 = rot[2]
				ZEDq4 = rot[3]

				# Update the UA Address Space
				ua_info_update()

			except:
				continue

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		
		rate.sleep()
