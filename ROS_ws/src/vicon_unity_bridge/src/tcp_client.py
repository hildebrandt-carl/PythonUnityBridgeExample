#!/usr/bin/env python3
import rospy
import socket
import numpy as np

from geometry_msgs.msg import TransformStamped


class TCPClient():

  def __init__(self):
    # When this node shuts down
    rospy.on_shutdown(self.shutdown_sequence)

    # Get the port number
    ip    = rospy.get_param('~ip_address', "127.0.0.1")
    port  = rospy.get_param('~port', 25000)
    topic = rospy.get_param('~topic', "")

    if len(topic) <= 0:
      print("Topic must be set")
      exit()

    # Holds the drone position and rotation
    self.position = np.array([0, 0, 0], dtype=float)
    self.rotation = np.array([0, 0, 0, 0], dtype=float)

    # Create the vicon subscriber
    print(f"Topic: {topic}")
    self.sub = rospy.Subscriber(f"{topic}", TransformStamped, self.data_callback)

    # Create the connection
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.sock.connect((ip, port))

    # Get the message size
    self.expected_msg_size = 8

    # Run the node
    self.Run()


  # This is the main loop of this class
  def Run(self):
     # Set the rate
    rate = rospy.Rate(10)

    # While running
    while not rospy.is_shutdown():

      # Convert both position and rotation a string and append them
      msg1 = ','.join(np.round(self.position,8).astype(str))
      msg2 = ','.join(np.round(self.rotation,8).astype(str))
      msg = msg1 + "," + msg2

      # Send the message and wait for a new one
      if self.sock is not None:
        self.sock.sendall(msg.encode("UTF-8"))

        # Create the byte array to hold the returned image
        byte_array = bytes()

        rospy.loginfo(f"Position: {self.position}")
        rospy.loginfo(f"Rotation: {self.rotation}")

        # Wait for the data
        while len(byte_array) < self.expected_msg_size:
          data = self.sock.recv(self.expected_msg_size * 2) # Set the max number of bytes to be 2X the length of what we expect
          byte_array += data

        rospy.loginfo(f"Received: {len(byte_array)} bytes")
        rospy.loginfo(f"=========================")

      # Sleep any excess time
      rate.sleep()


  # Call back to get the data
  def data_callback(self, msg):

    # Save the position with transform
    self.position[0] = msg.transform.translation.x
    self.position[1] = msg.transform.translation.z
    self.position[2] = msg.transform.translation.y

    # Convert Pose message to Euler angles
    self.rotation[0] = msg.transform.rotation.x
    self.rotation[1] = msg.transform.rotation.y
    self.rotation[2] = msg.transform.rotation.z
    self.rotation[3] = msg.transform.rotation.w

  # Shutdown code
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")
    self.sock.close()


def main():
  rospy.init_node("TCPClientNode")
  try:
    tcp_obj = TCPClient()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()