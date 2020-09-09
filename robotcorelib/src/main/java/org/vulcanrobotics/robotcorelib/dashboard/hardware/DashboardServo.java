package org.vulcanrobotics.robotcorelib.dashboard.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.dashboard.Dashboard;

public class DashboardServo {
    public double pos;
    public Servo servo;
    public int id;

    public DashboardServo() {}

    public DashboardServo(Servo servo, double pos) {
        this.servo = servo;
        this.pos = pos;
        Dashboard.addServo(this);
        id = Dashboard.getServos().indexOf(this);
    }

    public DashboardServo(Servo servo) {
        this.servo = servo;
        Dashboard.addServo(this);
        id = Dashboard.getServos().indexOf(this);
    }


    public void update() {
        Dashboard.sendToDash("/update Servo " + id + " " + servo.getPosition());
    }


    public Servo getServo() {
        return servo;
    }
}
