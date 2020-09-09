package org.vulcanrobotics.robotcorelib.dashboard.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.vulcanrobotics.robotcorelib.dashboard.Dashboard;

public class DashboardMotor {
    public int id;
    public int limLow;
    public int limHigh;

    private DcMotor motor;

    public DashboardMotor() {}

    //the dashboardmotor id system is shitty, should rework for auto ids
    public DashboardMotor(DcMotor motor) {
        this.motor = motor;
        this.id = Dashboard.getMotors().indexOf(this);
        Dashboard.addMotor(this);
    }

    public DashboardMotor(DcMotor motor, int limLow, int limHigh) {

        this.motor = motor;
        this.limLow = limLow;
        this.limHigh = limHigh;
        this.id = Dashboard.getMotors().indexOf(this);
        Dashboard.addMotor(this);
    }

    public void update() {
        int pos = motor.getCurrentPosition();

        String[] updates = new String[] {
                "/set DcMotor " + id + " position " + pos,
                "/set DcMotor " + id + " power " + motor.getPower()
        };

        Dashboard.sendToDash(updates);

    }

    public void setLowLimit(int limLow) {
        this.limLow = limLow;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setHighLimit(int limHigh) {
        this.limHigh = limHigh;
    }

    public DcMotor getMotor() {
        return motor;
    }

}
