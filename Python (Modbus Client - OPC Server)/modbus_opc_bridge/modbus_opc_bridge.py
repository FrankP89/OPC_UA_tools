"""
This code enables the communication from the KUKA iiwa controller to any OPC-UA clients.

This development was performed on the 28th of July, 2020.
Maintainer and creator: Walter Frank Pintor Ortiz, walterpintor@gmail.com
"""

"""
At this stage, the code only allows users to monitor Joint values.
End-users may add more registers depending on the needs.

TODO: Indicate how many registers to read when executing this program
"""

import getopt
import os
import platform
import subprocess
import sys
import time
import struct

from threading import Thread, Lock
from opcua import Server, ua
from pyModbusTCP.client import ModbusClient
from pyModbusTCP import utils

print('Welcome to the Modbus/OPC-UA bridge for KUKA iiwa \nFor defined IP addresses, utilize the following arguments')
print('modbus_opc_bridge.py --opc_ip <opc_ip> --opc_port <opc_port> --modbus_ip <mir_ip> --modbus_port <mir_port>')
time.sleep(2)

##############################################################################
ip_opc_server_address = "192.168.1.84"
opc_port_server_no = 4840

ip_modbus = "127.0.0.1"
port_modbus = 10030
##############################################################################

##############################################################################
# Global Variables
myvar = []

# Modbus regs - set global
regs = []

# init a thread lock
regs_lock = Lock()
##############################################################################


##############################################################################
########## Check input function ##############################################
##############################################################################

def check_arguments(argv):
    global ip_opc_server_address
    global opc_port_server_no
    global ip_modbus
    global port_modbus
    opc_ip_read = ip_opc_server_address
    opc_port_read = opc_port_server_no
    modbus_ip_read = ip_modbus
    modbus_port_read = port_modbus

    try:
        opts, args = getopt.getopt(argv, "hi:o:", ["opc_ip=", "opc_port=", "modbus_ip=", "modbus_port="])
    except getopt.GetoptError:
        print('modbus_opc_bridge.py --opc_ip <opc_ip> --opc_port <opc_port> --modbus_ip <modbus_ip> --modbus_port <modbus_port>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('modbus_opc_bridge.py --opc_ip <opc_ip> --opc_port <opc_port> --modbus_ip <modbus_ip> --modbus_port <modbus_port>')
            sys.exit()
        elif opt in ("-opc_ip", "--opc_ip"):
            opc_ip_read = str(arg)
        elif opt in ("-opc_port", "--opc_port"):
            opc_port_read = arg
        elif opt in ("-modbus_ip", "--modbus_ip"):
            modbus_ip_read = str(arg)
        elif opt in ("-modbus_port", "--modbus_port"):
            modbus_port_read = arg

    print('OPC-UA IP provided is: ', opc_ip_read)
    print('The OPC-UA port is "', opc_port_read)
    print('Modbus IP provided is: ', modbus_ip_read)
    print('The Modbus port is "', modbus_port_read)
    ip_opc_server_address = opc_ip_read
    opc_port_server_no = opc_port_read
    ip_modbus = modbus_ip_read
    port_modbus = modbus_port_read


##############################################################################
########## End of Check input function #######################################
##############################################################################


##############################################################################
########## Connect functions #################################################
##############################################################################

################################
### Connect Modbus functions ###
################################

def start_modbus_client(modbus_ip, modbus_port):
    try:
        # TCP auto connect on first Modbus request
        client = ModbusClient(host=modbus_ip, port=modbus_port, auto_open=True)
        # managing TCP sessions with call to c.open()/c.close()
        client.open()
    except:
        time.sleep(2)
        print("Retrying to establish Modbus client...")
        start_modbus_client(modbus_ip, modbus_port)

    print("Successfully established Modbus client!")
    return client


def read_robot_regs(client):
    global regs  
    # polling loop
    while True:
        # keep TCP open
        if not client.is_open():
            client.open()
        # do modbus reading on socket
        reg_list = client.read_holding_registers(0, 14)
        # if read is ok, store result in regs (with thread lock synchronization)
        if reg_list:
            with regs_lock:
                regs = list(reg_list)
        # 1s before next polling
        time.sleep(1)


################################
### Connect OPC-UA functions ###
################################

def create_opc_server():
    global myvar
    # setup our server
    server = Server()
    server.set_endpoint("opc.tcp://" + ip_opc_server_address + ":" + str(opc_port_server_no) + "/freeopcua/server/")

    # setup our own namespace, not really necessary but should as spec
    uri = "http://examples.freeopcua.github.io"
    idx = server.register_namespace(uri)

    # get Objects node, this is where we should put our nodes
    objects = server.get_objects_node()

    # Add as needed - address space
    myvar = [None] * 30
    myobj = objects.add_object(idx, "KUKA_Joints")
    myvar[0] = myobj.add_variable(idx, "fGet_Joint_1", 0.0, ua.VariantType.Float)
    myvar[1] = myobj.add_variable(idx, "fGet_Joint_2", 0.0, ua.VariantType.Float)
    myvar[2] = myobj.add_variable(idx, "fGet_Joint_3", 0.0, ua.VariantType.Float)
    myvar[3] = myobj.add_variable(idx, "fGet_Joint_4", 0.0, ua.VariantType.Float)
    myvar[4] = myobj.add_variable(idx, "fGet_Joint_5", 0.0, ua.VariantType.Float)
    myvar[5] = myobj.add_variable(idx, "fGet_Joint_6", 0.0, ua.VariantType.Float)
    myvar[6] = myobj.add_variable(idx, "fGet_Joint_7", 0.0, ua.VariantType.Float)
        

    for variables in range(len(myvar)):
        if myvar[variables] is not None:
            print(variables)
            myvar[variables].set_writable()  # Set MyVariable to be writable by clients
            time.sleep(0.05)

    try:
        # Starting OPC-UA Server!
        server.start()
        print("OPC-UA server started!")
    except:
        # Retry opening
        create_opc_server()
        print("Retrying to start OPC-UA server...")
        time.sleep(2)

    return server


##############################################################################
########## End of Connect functions ##########################################
##############################################################################


def write_opc_space(opc_server, modbus_client):
    global myvar
    final_val = []
    # Start polling thread
    modbus_poll = Thread(target=read_robot_regs, args= (modbus_client,))
    # set daemon: polling thread will exit if main thread exit
    modbus_poll.daemon = True
    modbus_poll.start()
    
    try:      
        while True:

            # Print regs list (with thread lock synchronization)
            with regs_lock:
                print("Modbus (Holding) registers: ", regs)         

            for i in range(len(myvar)):
                if myvar[i] is not None:      
                    if regs:
                        # Combine regs. Use the last reg in the first arg.
                        mixed_regs = struct.pack('>HH', regs[(i*2)+1], regs[(i*2)])
                        final_val.append(struct.unpack('>f', mixed_regs)[0])
                        
                        dv = ua.DataValue(ua.Variant(final_val[i], ua.VariantType.Float))
                        myvar[i].set_value(dv)
                        print(myvar[i].get_browse_name(), ", ", myvar[i].get_value())
                        time.sleep(0.2)            
            
            # 1 sec before next print
            time.sleep(0.5)
            
    finally:
        # close connection, remove subscriptions, etc
        print("Closing connections")
        opc_server.stop()
        modbus_client.close()


if __name__ == "__main__":
    # Check if arguments were given at the start
    check_arguments(sys.argv[1:])

    # Start OPC-UA Server
    opc_server = create_opc_server()

    # Starting Modbus Client!
    modbus_client = start_modbus_client(modbus_ip=ip_modbus, modbus_port=port_modbus)

    # Write to OPC-UA variables
    write_opc_space(opc_server, modbus_client)
