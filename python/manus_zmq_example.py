import zmq

'''
This is a minimal Python example that talks to the MANUS SDK and prints the data out directly to the terminal.
To run this, first run the MANUS SDK and then this script.

'''


context = zmq.Context()
# Socket to talk to Manus SDK
print("Connecting to SDK")
socket = context.socket(zmq.PULL)
socket.setsockopt(zmq.CONFLATE, True)     
socket.connect("tcp://127.0.0.1:8000") # Set IP and port to receive data from MANUS SDK


while True:
    # Wait for message
    message = socket.recv()
    # Receive the message from the socket
    message = message.decode('utf-8')
    data = message.split(",")   
    print(data)

