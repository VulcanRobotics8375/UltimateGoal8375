package org.vulcanrobotics.robotcorelib.dashboard;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;

/**
 * The Client is the backend class for connecting the phone to the Dashboard as a client.
 * The CLient is it's own thread so that all Dashboard operations are asynchronous from the internal robot commands.
 * @see java.lang.Thread
 */
public class Client extends Thread {
    /**
     * The IP of the Server, local IP of the computer running VulcanDashboard
     */
    private String ip;
    /**
     * Predefined port from the VulcanDash Server.
     */
    private int port;

    /**
     * The Socket that connects to the server as a client
     * @see Socket
     */
    Socket echoSocket;

    private volatile boolean stop = false;

    /**
     * Data streams that open up a space in memory for transferring data to and from VulcanDashboard.
     */
    private DataInputStream inputStream;
    private DataOutputStream outputStream;

    /**
     * Default Constructor.
     * @param ip The local IP of the server host
     * @param port the predefined port of the server host
     */
    public Client(String ip, int port) {
        super();
        this.ip = ip;
        this.port = port;
    }

    /**
     * Starts a thread that connects to the server, initializes the dataStreams,
     * and runs a message receiver until <code>killProcess()</code> is called.
     */
    public void run() {
        try {
            echoSocket = new Socket(ip, port);

            inputStream = new DataInputStream(echoSocket.getInputStream());
            outputStream = new DataOutputStream(echoSocket.getOutputStream());

            while(!stop) {
                MessageHandler.parseMessage(inputStream.readUTF());
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Sends a string to the Dashboard server.
     * @param msg Data that is sent to the dashboard.
     */
    public void send(String msg) {
        try {
            outputStream.writeUTF(msg);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Deprecated
    public String get() throws IOException {
        return inputStream.readUTF();
    }

    /**
     * makes the volatile boolean <code>stop</code> true, ending the client thread.
     */
    public void killProcess() {
        stop = true;
    }

}
