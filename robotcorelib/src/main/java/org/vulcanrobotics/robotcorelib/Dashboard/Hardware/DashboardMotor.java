package org.vulcanrobotics.robotcorelib.Dashboard.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.vulcanrobotics.robotcorelib.Dashboard.Dashboard;

public class DashboardMotor {
    public int id;
    public int limLow;
    public int limHigh;

    private DcMotor motor;

    public DashboardMotor() {}

    public DashboardMotor(DcMotor motor, int id) {
        this.motor = motor;
        this.id = id;
        Dashboard.addMotor(this);
    }

    public DashboardMotor(int id) {
        this.id = id;
        Dashboard.addMotor(this);
    }

    public DashboardMotor(DcMotor motor, int id, int limLow, int limHigh) {

        this.motor = motor;
        this.id = id;
        this.limLow = limLow;
        this.limHigh = limHigh;
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
