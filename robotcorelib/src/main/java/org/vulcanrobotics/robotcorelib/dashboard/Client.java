package org.vulcanrobotics.robotcorelib.dashboard;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;

public class Client extends Thread {
    private String ip;
    private int port;
    Socket echoSocket;

    private DataInputStream inputStream;
    private DataOutputStream outputStream;

    public Client(String ip, int port) {
        super();
        this.ip = ip;
        this.port = port;
    }

    public void run() {
        try {
            echoSocket = new Socket(ip, port);

            inputStream = new DataInputStream(echoSocket.getInputStream());
            outputStream = new DataOutputStream(echoSocket.getOutputStream());

            while(true) {
                MessageHandler.parseMessage(inputStream.readUTF());
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void send(String msg) {
        try {
            outputStream.writeUTF(msg);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public String get() throws IOException {
        return inputStream.readUTF();
    }

}
