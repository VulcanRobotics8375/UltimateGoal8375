package org.vulcanrobotics.robotcorelib.dashboard;

import org.vulcanrobotics.robotcorelib.dashboard.hardware.DashboardMotor;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.DashboardServo;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.Sensor;
import org.vulcanrobotics.robotcorelib.math.Point;

import java.util.ArrayList;
import java.util.List;

/**
 * The main class for Dashboard.
 * this is where we initialize and relay all the necessary data to the Client.
 */
public class Dashboard {
    public static String ip;
    public static int port;
    private static Client client;
    public static volatile boolean running = false;
    public static Point robotPos = new Point();
    public static double robotAngle;
    public static volatile boolean enabled;

    private static List<DashboardMotor> motors = new ArrayList<>();
    private static List<DashboardServo> servos = new ArrayList<>();
    private static List<Constant> constants = new ArrayList<>();
    private static List<Sensor> sensors = new ArrayList<>();
    private static List<TelemetryMsg> messages = new ArrayList<>();

    public static void connect() {
        client = new Client(ip, port);

        client.start();
    }

    public static void connect(String ip, int port) {
        client = new Client(ip, port);

        client.start();
    }

    public static void sendToDash(String msg) {
        client.send(msg);


    }

    public static void sendToDash(String[] msgs) {
        for (String msg : msgs) {
            sendToDash(msg);
        }

    }

    public static void setServerParams(String ip, int port) {
        Dashboard.ip = ip;
        Dashboard.port = port;
    }

    public static void addMotor(DashboardMotor motor) {
        motors.add(motor);
        sendToDash("/add DcMotor " + motor.id);
    }

    public static void addServo(DashboardServo servo) {
        servos.add(servo);
        sendToDash("/add Servo " + servo.id);
    }

    public static void addConstant(Constant constant) {
        constants.add(constant);
        sendToDash("/add Constant " + constant.key + " " + constant.val + " double");
    }

    public static void addSensor(Sensor sensor) {
        sensors.add(sensor);
        sendToDash("/add Sensor " + sensor.name);
    }

    public static void addMessage(TelemetryMsg msg) {
        messages.add(msg);
        sendToDash("/telemetry " + msg.id + " " + msg.getMessage());
    }

    public static List<DashboardMotor> getMotors() {

        return motors;
    }

    public static List<DashboardServo> getServos() {
        return servos;
    }

    public static void start() {
        sendToDash("/start");
        running = true;
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(running) {
                    for (DashboardMotor motor : motors) {
                        motor.update();
                    }
                    for (DashboardServo servo : servos) {
                        servo.update();
                    }
                    for (Sensor sensor : sensors) {
                        sensor.update();
                    }
                    sendToDash("/update Robot " + robotPos.x + " " + robotPos.y + " " + robotAngle);
                }
                client.killProcess();
            }
        }).start();
    }

    public List<Constant> getConstants() {
        return constants;
    }

}
