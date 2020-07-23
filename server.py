"""
A set of tools (adapted examples) to quickly check the status of clients or servers.

This development was performed on the 23rd of July, 2020.
Maintainer and creator: Walter Frank Pintor Ortiz, walterpintor@gmail.com
https://github.com/FreeOpcUa/python-opcua/blob/master/examples/server-minimal.py
"""

import sys
sys.path.insert(0, "..")
import time
from opcua import Server, ua

myvar = []


if __name__ == "__main__":

    # setup our server
    server = Server()
    server.set_endpoint("opc.tcp://127.0.0.1:4841/freeopcua/server/")

    # setup our own namespace, not really necessary but should as spec
    uri = "http://examples.freeopcua.github.io"
    idx = server.register_namespace(uri)

    # get Objects node, this is where we should put our nodes
    objects = server.get_objects_node()

    # Add as needed - address space
    myvar = [None] * 30
    myobj = objects.add_object(idx, "MiRVariables")
    myvar[0] = myobj.add_variable(idx, "sSet_send_action_to_AMR", "")
    myvar[1] = myobj.add_variable(idx, "bGet_AMR_status", False)
    myvar[2] = myobj.add_variable(idx, "fGet_AMR_battery_life", 0.0)
    myvar[3] = myobj.add_variable(idx, "fGet_AMR_pos_x", 0.0)
    myvar[4] = myobj.add_variable(idx, "fGet_AMR_pos_y", 0.0)
    myvar[5] = myobj.add_variable(idx, "fGet_AMR_pos_theta", 0.0)
    myvar[6] = myobj.add_variable(idx, "fGet_AMR_imu_orient_x", 0.0)
    myvar[7] = myobj.add_variable(idx, "fGet_AMR_imu_orient_y", 0.0)
    myvar[8] = myobj.add_variable(idx, "fGet_AMR_imu_orient_z", 0.0)
    myvar[9] = myobj.add_variable(idx, "fGet_AMR_imu_orient_w", 0.0)
    myvar[10] = myobj.add_variable(idx, "fGet_AMR_imu_ang_vel_x", 0.0)
    myvar[11] = myobj.add_variable(idx, "fGet_AMR_imu_ang_vel_y", 0.0)
    myvar[12] = myobj.add_variable(idx, "fGet_AMR_imu_ang_vel_z", 0.0)
    myvar[13] = myobj.add_variable(idx, "fGet_AMR_imu_lin_acc_x", 0.0)
    myvar[14] = myobj.add_variable(idx, "fGet_AMR_imu_lin_acc_y", 0.0)
    myvar[15] = myobj.add_variable(idx, "fGet_AMR_imu_lin_acc_z", 0.0)
    myvar[16] = myobj.add_variable(idx, "fGet_AMR_odom_pose_lin_x", 0.0)
    myvar[17] = myobj.add_variable(idx, "fGet_AMR_odom_pose_lin_y", 0.0)
    myvar[18] = myobj.add_variable(idx, "fGet_AMR_odom_pose_lin_z", 0.0)
    myvar[19] = myobj.add_variable(idx, "fGet_AMR_odom_twist_orien_x", 0.0)
    myvar[20] = myobj.add_variable(idx, "fGet_AMR_odom_twist_orien_y", 0.0)
    myvar[21] = myobj.add_variable(idx, "fGet_AMR_odom_twist_orien_z", 0.0)

    myvar[22] = myobj.add_variable(idx, "sSet_action_to_AMR", "")
    myvar[23] = myobj.add_variable(idx, "bSet_AMR_e_stop", False)
    myvar[24] = myobj.add_variable(idx, "fSet_AMR_pos_x", 0.0)
    myvar[25] = myobj.add_variable(idx, "fSet_AMR_pos_y", 0.0)
    myvar[26] = myobj.add_variable(idx, "fSet_AMR_pos_theta", 0.0)
    myvar[27] = myobj.add_variable(idx, "bSet_pause_AMR", False)
    myvar[28] = myobj.add_variable(idx, "bSet_ready_AMR", False)


    for variables in range(len(myvar)):
        if myvar[variables] is not None:
            print variables
            myvar[variables].set_writable()  # Set MyVariable to be writable by clients
            time.sleep(0.05)   

    # starting!
    server.start()
    

    try:
        count = 0
        while True:
            time.sleep(1)
            for i in range(len(myvar)):
                if myvar[i] is not None:
                    print (myvar[i].get_display_name.__str__ , ", " , myvar[i].get_value())
                    time.sleep(0.2)
    finally:
        # close connection, remove subcsriptions, etc
        print("Closing connection")
        server.stop()
