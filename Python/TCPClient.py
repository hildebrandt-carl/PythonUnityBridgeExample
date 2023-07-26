#!/usr/bin/env python3
import time
import socket
import argparse
import numpy as np


class TCPClient():

  def __init__(self, ip="127.0.0.1", port=25000):

    # Holds the drone position and rotation
    self.position = np.array([0, 0, 0], dtype=float)
    self.rotation = np.array([0, 0, 0, 0], dtype=float)

    # Create the connection
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.sock.connect((ip, port))

    # Each time we send a message we expect the word "Accepted" to be returned
    self.expected_msg_size = 8

    # Set the rate the data is sent at
    self.rate = 10
    self.freq = 1.0/self.rate

    # Run the node
    self.Run()

  # This is the main loop of this class
  def Run(self):

    # Keep running
    while True:
      
      # We want to update at a set frequency
      start_time = time.time() 
      
      # Convert both position and rotation a string and append them
      msg1 = ','.join(np.round(self.position,8).astype(str))
      msg2 = ','.join(np.round(self.rotation,8).astype(str))
      msg = msg1 + "," + msg2

      # Send the message and wait for a new one
      if self.sock is not None:
        self.sock.sendall(msg.encode("UTF-8"))

        # Create the byte array to hold the returned image
        byte_array = bytes()

        # In ROS this would be done in a callback function
        self.position[0] += 0.1

        print(f"Position: {self.position}")
        print(f"Rotation: {self.rotation}")

        # Wait for the data
        while len(byte_array) < self.expected_msg_size:
          data = self.sock.recv(self.expected_msg_size * 2) # Set the max number of bytes to be 2X the length of what we expect
          byte_array += data

        print(f"Received: {len(byte_array)} bytes")
        print(f"=========================")

      # Sleep for the remaining duration of self.freq
      time.sleep(max(self.freq - (time.time() - start_time), 0))


if __name__ == '__main__':

  # Create the parser
  parser = argparse.ArgumentParser()

  # Add the arguments
  parser.add_argument("--ip", type=str, default="127.0.0.1", help="The IP address of the TCP server")
  parser.add_argument("--port", type=int, default=25000, help="The port of the TCP server")

  # Parse the arguments
  args = parser.parse_args()

  # Run the code
  tcp_obj = TCPClient(ip=args.ip, port=args.port)