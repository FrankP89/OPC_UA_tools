"""
This code enables the communication from the KUKA iiwa controller to the OPC-UA server.

This development was performed on the 28th of July, 2020.
Maintainer and creator: Walter Frank Pintor Ortiz, walterpintor@gmail.com
"""

import getopt
import os
import platform
import subprocess
import sys
import time

from opcua import Server, ua
from pyModbusTCP.client import ModbusClient

print('Welcome to the Modbus/OPC-UA bridge for KUKA iiwa \nFor defined IP addresses, utilize the following arguments')
print('modbus_opc_bridge.py --opc_ip <opc_ip> --opc_port <opc_port> --modbus_ip <mir_ip> --modbus_port <mir_port>')
time.sleep(2)

##############################################################################
ip_opc_server_address = "192.168.1.84"
opc_port_server_no = 4840

ip_modbus = "127.0.0.1"
port_modbus = 502
##############################################################################

##############################################################################
# Global Variables
myvar = []


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

# Pinging function to check if AMR is there #
def ping(host):
    """
    Returns True if host (str) responds to a ping request.
    Remember that a host may not respond to a ping (ICMP) request even if the host name is valid.
    Answer provided in: https://stackoverflow.com/questions/2953462/pinging-servers-in-python and
    adapated to Windows.
    """

    # Option for the number of packets as a function of
    param = '-n' if platform.system().lower() == 'windows' else '-c'

    # Building the command. Ex: "ping -c 1 google.com"
    command = ['ping', param, '1', host]

    if platform.system().lower() == 'windows':
        check = subprocess.Popen(["ping.exe", host], stdout=subprocess.PIPE).communicate()[0]
    else:
        print("Pinging...".format(host))
        check = os.system("ping -c 1 " + host)

        # check = subprocess.Popen(["ping", host], stdout=subprocess.PIPE).communicate()[0]

    if ('unreachable' in str(check)) or (check == 1):
        return 0
    else:
        return subprocess.call(command) == 0


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
    # Read registers that have robot position
    regs = client.read_holding_registers(0, 15)
    if regs:
        print(regs)
    else:
        print("read error")
    return regs


################################
### Connect OPC-UA functions ###
################################

def create_opc_server():
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
    myobj = objects.add_object(idx, "KUKAVariables")
    myvar[0] = myobj.add_variable(idx, "sSet_send_action_to_AMR", "")
    myvar[1] = myobj.add_variable(idx, "bGet_AMR_status", False)
    myvar[2] = myobj.add_variable(idx, "fGet_AMR_battery_life", 0.0, ua.VariantType.Float)
    myvar[3] = myobj.add_variable(idx, "fGet_AMR_pos_x", 0.0, ua.VariantType.Float)
    myvar[4] = myobj.add_variable(idx, "fGet_AMR_pos_y", 0.0, ua.VariantType.Float)
    myvar[5] = myobj.add_variable(idx, "fGet_AMR_pos_theta", 0.0, ua.VariantType.Float)
    myvar[6] = myobj.add_variable(idx, "fGet_AMR_imu_orient_x", 0.0, ua.VariantType.Float)

    for variables in range(len(myvar)):
        if myvar[variables] is not None:
            print(variables)
            myvar[variables].set_writable()  # Set MyVariable to be writable by clients
            time.sleep(0.05)

    try:
        # Starting OPC-UA Server!
        server.start()
    except:
        # Retry opening
        create_opc_server()
        print("Retrying to start OPC-UA server...")
        time.sleep(2)

    return server


def write_opc_space(opc_server, modbus_client):
    while True:
        try:
            count = 0
            while True:
                time.sleep(1)
                KUKAregs = read_robot_regs(modbus_client)
                for i in range(len(myvar)):
                    if myvar[i] is not None:
                        print(myvar[i].get_browse_name(), ", ", myvar[i].get_value())
                        time.sleep(0.2)
        finally:
            # close connection, remove subscriptions, etc
            print("Closing connection")
            opc_server.stop()
            modbus_client.close()
            break


if __name__ == "__main__":
    # Check if arguments were given at the start
    check_arguments(sys.argv[1:])

    # Start OPC-UA Server
    opc_server = create_opc_server()

    # Starting Modbus Client!
    modbus_client = start_modbus_client(modbus_ip=ip_modbus, modbus_port=port_modbus)

    # Write to OPC-UA variables
    write_opc_space(opc_server, modbus_client)
