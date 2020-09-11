package org.vulcanrobotics.robotcorelib.dashboard.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Not a dashboard hardware item yet, but is very similar to DashboardMotor in a lot of ways
 * Key Differences-- Odometer is not made to be a DcMotor, but rather the encoder port of a DcMotor.
 * This is why we have a DcMotor object, but no way to access it outside of this class.
 * This is because the only thing we need from the odometer is its position in encoder ticks.
 */
public class Odometer {

    private DcMotor pod;
    /**
     * Odometers use a manual naming system rather than ID's,
     * since this property is used by more than just the dashboard.
     */
    private String name;

    /**
     * Odometer uses the same hardwareMap as the motor port that it is attached to.
     * Though we could just use getCurrentPosition of that motor port with DashboardMotor, this separates the definitions much more clearly.
     **/
    public Odometer(DcMotor pod) {
        this.pod = pod;
    }

    public Odometer(String name, DcMotor pod) {
        this.name = name;
        this.pod = pod;
    }

    /**
     *
     * @return the odometers current position
     */
    public double getPosition() {
        return pod.getCurrentPosition();
    }

    public String getName() {
        return name;
    }

}
