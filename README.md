# Python Unity Bridge Example

This project provides an implementation of a TCP server in Unity3D and a TCP client in Python. The purpose of this system is to facilitate real-time control of Unity GameObjects from an external Python script.

![Example](example.gif)

## Description

The Unity TCP server attaches to a GameObject and listens for incoming connections. On the other side, the Python TCP client connects to the Unity server, sending a series of (x,y,z) coordinates and (x,y,z,w) quaternions. This data is then used to update the position and rotation of the GameObject in the Unity scene. The system can be used to programmatically control GameObjects for a wide variety of applications, such as simulations, AI training environments, robotics, and more.

This system is designed to be simple and flexible, capable of being extended for various use-cases. The Python client also includes features like variable send rates for tuning performance to match specific needs.

While the Unity server runs on a separate thread to keep the main game loop responsive, the Python client maintains a consistent update rate to ensure smooth GameObject movements. When a new position is received, the Unity server responds with an "Accepted" message, allowing for simple, reliable two-way communication. 

Please refer to the provided examples to understand how to use and implement this system in your own Unity projects. Feedback and contributions are welcomed!

## Running the Code

To run this code load the unity project and hit play.

Then run the python in the `Python` folder using:
```python
python3 TCPClient.py # For Ubuntu
python .\TCPClient.py # For Windows
```

Note the default IP address is set to `127.0.0.1` however, I have tested running the client and server on different machines within the same network. Just change the IP address in both the server and client script and you will be good to go.

