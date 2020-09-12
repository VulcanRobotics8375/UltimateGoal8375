package org.vulcanrobotics.robotcorelib.dashboard.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.vulcanrobotics.robotcorelib.dashboard.Dashboard;

/**
 * DashboardMotor is essentially a DcMotor, but it has the ability to interface with VulcanDashboard.
 * @see DcMotor
 */
public class DashboardMotor {
    /**
     * ID for the dashboard system to identify this motor.
     * ID Number is automatically assigned on creation.
     */
    public int id;
    public int limLow;
    public int limHigh;

    /**
     * The Motor attached to DashboardMotor so we can have a two way communication between the robot and the dashboard for each Motor.
     * The robot updates the motor positions/power, and the dashboard pulls that data.
     */
    private DcMotor motor;

    public DashboardMotor() {}

    /**
     * Constructor for the DashboardMotor that includes the auto-ID system.
     * @param motor the DcMotor linked to this dashboardMotor.
     */
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

    /**
     * update when the Dashboard is querying for it.
     * this sends a list of commands that update the motors position and power on the Dashboard.
     */
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

    /**
     *
     * @return the DcMotor attached to this object, so we can interact with it like a normal DcMotor.
     */
    public DcMotor getMotor() {
        return motor;
    }

}
