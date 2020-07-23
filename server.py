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
    myobj = objects.add_object(idx, "MyObject")
    myvar = myobj.add_variable(idx, "MyVariable", 6.7)
    myvar.set_writable()  # Set MyVariable to be writable by clients

    # starting!
    server.start()

    try:
        count = 0
        while True:
            time.sleep(1)
            count += 0.1
            myvar.set_value(count)
    finally:
        # close connection, remove subcsriptions, etc
        server.stop()
