package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorExample extends Subsystem {

    private Rev2mDistanceSensor distanceSensor;

    private boolean state;
    private int counter = 0;

    @Override
    public void init() {
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");
    }

    /**
     * counts every time the distance sensor returns a value under 10cm.
     */
    public void run() {
        if(distanceSensor.getDistance(DistanceUnit.CM) < 10 && !state) {
            counter++;
            state = true;
        }

        if(distanceSensor.getDistance(DistanceUnit.CM) > 10 && state) {
            state = false;
        }

    }

    public int getCounts() {
        return counter;
    }

    @Override
    public void stop() {}

}
