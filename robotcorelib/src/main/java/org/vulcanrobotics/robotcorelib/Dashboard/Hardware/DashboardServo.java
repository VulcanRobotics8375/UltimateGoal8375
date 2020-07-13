package org.vulcanrobotics.robotcorelib.Dashboard.Hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.Dashboard.Dashboard;

public class DashboardServo {
    public double pos;
    public Servo servo;
    public int id;

    public DashboardServo() {}

    public DashboardServo(Servo servo, double pos, int id) {
        this.servo = servo;
        this.pos = pos;
        this.id = id;
    }

    public DashboardServo(Servo servo, int id) {
        this.servo = servo;
        this.id = id;
    }

    public DashboardServo(int id) {
        this.id = id;
    }

    public void update() {
        Dashboard.sendToDash("/update Servo " + id + " " + servo.getPosition());
    }
}
