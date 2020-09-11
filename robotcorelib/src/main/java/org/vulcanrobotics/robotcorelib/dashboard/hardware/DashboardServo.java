package org.vulcanrobotics.robotcorelib.dashboard.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.dashboard.Dashboard;

/**
 * A Servo class that is used to interface servos with the dashboard.
 */
public class DashboardServo {
    /**
     * Servo Position, updated automatically whenever needed.
     * Don't rely on this for position, instead use getServo().getPosition()
     */
    public double pos;
    /**
     * The Servo Object attached to this Object.
     */
    public Servo servo;
    /**
     * Automatically assigned ID Number.
     */
    public int id;

    public DashboardServo() {}

    public DashboardServo(Servo servo, double pos) {
        this.servo = servo;
        this.pos = pos;
        Dashboard.addServo(this);
        id = Dashboard.getServos().indexOf(this);
    }

    /**
     * Main Constructor, handles auto ID and Servo binding
     * @param servo attaches a Servo object to this Object.
     */
    public DashboardServo(Servo servo) {
        this.servo = servo;
        Dashboard.addServo(this);
        id = Dashboard.getServos().indexOf(this);
    }

    /**
     * sends a command to the dashboard to update the servo position.
     */
    public void update() {
        Dashboard.sendToDash("/update Servo " + id + " " + servo.getPosition());
    }

    /**
     *
     * @return the servo attached to this Object.
     */
    public Servo getServo() {
        return servo;
    }
}
