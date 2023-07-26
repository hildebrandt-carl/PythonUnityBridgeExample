using System;
using System.IO;
using System.Net;
using System.Text;
using System.Threading;
using System.Net.Sockets;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

using UnityEngine;


public class TCPServer : MonoBehaviour
{

    // Create the public interface
    public string connectionIP = "127.0.0.1" ;
    public int connectionPort = 25000 ;

    // Variables used to create the TCP server
    private Thread serverThread;
    private TcpClient tcpClient;
    private TcpListener tcpServer;
    private IPAddress localIPAddress;

    // Variable used to hold the drone Pose
    private Vector3 dronePosition;
    private Quaternion droneRotation;

    // Keep track of the state of the TCP server
    private bool serverRunning = false;

    // Keep track of time running
    private float currentTimeRunning;

    // Called at the start
    private void Start()
    {

        Debug.Log("Starting on Port " + connectionPort.ToString());
        // Get the start time of the program
        currentTimeRunning = 0 ;

        // Initialize the drone position
        dronePosition = Vector3.zero; 
        droneRotation.eulerAngles = Vector3.zero; 

        // Start the TCP Server
        ThreadStart ts = new ThreadStart(RunTCPServer);
        serverThread = new Thread(ts);
        serverThread.Start();
    }


    // Called each frame
    private void Update()
    {
        // Set the drones position and rotation
        transform.position = dronePosition;
        transform.rotation = droneRotation;

        // Update the current time
        currentTimeRunning += Time.deltaTime;

    }

    // Starts and runs the TCP Server
    private void RunTCPServer()
    {
        // Start the TCP server
        localIPAddress = IPAddress.Parse(connectionIP);
        tcpServer = new TcpListener(localIPAddress, connectionPort);
        tcpServer.Start();

        // Wait to accept clients
        Debug.Log("Waiting for clients");
        tcpClient = tcpServer.AcceptTcpClient();
        Debug.Log("Clients connected");

        // Start listening to the client
        serverRunning = true;
        while (serverRunning)
        {
            CommunicateWithClient();
        }

        // When we are done, stop the server
        tcpServer.Stop();
    }

    // Communicates with the client
    private void CommunicateWithClient()
    {
        // Get the TCP client stream
        NetworkStream stream = tcpClient.GetStream();
        byte[] buffer = new byte[tcpClient.ReceiveBufferSize];

        // Read the data from the new stream and convert to string
        int bytesRead = stream.Read(buffer, 0, tcpClient.ReceiveBufferSize);
        string dataReceived = Encoding.UTF8.GetString(buffer, 0, bytesRead); 

        // Check we have data
        if (dataReceived != null)
        {

            // Convert to a new position
            getNewDronePose(dataReceived);

            // Respond with a message
            string serverResponse = "Accepted";
            byte[] serverResponseBytes = Encoding.UTF8.GetBytes(serverResponse);
            stream.Write(serverResponseBytes, 0, serverResponseBytes.Length);
        }

    }

    // Convert a string to a Vector 3
    private void getNewDronePose(string msg)
    {
        // split the items
        // Debug.Log(msg);
        string[] msgArray = msg.Split(',');

        // Store as a Vector3
        Vector3 newPosition = new Vector3(
            float.Parse(msgArray[0]),
            float.Parse(msgArray[1]),
            float.Parse(msgArray[2]));

        // Get the new rotation
        Quaternion newRotation = new Quaternion(
            float.Parse(msgArray[3]),
            float.Parse(msgArray[4]),
            float.Parse(msgArray[5]),
            float.Parse(msgArray[6]));

        // Update the drone position and rotation
        dronePosition = newPosition;
        droneRotation = newRotation;

        return;
    }
}