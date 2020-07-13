package org.vulcanrobotics.robotcorelib.Dashboard.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometer {

    private DcMotor pod;
    private String name;

    /**
     * Odometer uses the same hardwareMap as the motor port that it is attached to.
     * Though we could just use getCurrentPosition of that motor port, this separates the definitions much more clearly.
     **/
    public Odometer(DcMotor pod) {
        this.pod = pod;
    }

    public Odometer(String name, DcMotor pod) {
        this.name = name;
        this.pod = pod;
    }

    public double getPosition() {
        return pod.getCurrentPosition();
    }

    public String getName() {
        return name;
    }

}
