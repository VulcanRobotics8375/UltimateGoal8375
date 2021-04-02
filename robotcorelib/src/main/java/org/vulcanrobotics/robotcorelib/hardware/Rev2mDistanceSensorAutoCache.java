package org.vulcanrobotics.robotcorelib.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Rev2mDistanceSensorAutoCache {

    private Rev2mDistanceSensor sensor;
    private ElapsedTime timer;
    private double cachedValue;
    private DistanceUnit distanceUnit = DistanceUnit.CM;

    public Rev2mDistanceSensorAutoCache(Rev2mDistanceSensor sensor) {
        this.sensor = sensor;
    }
    public Rev2mDistanceSensorAutoCache(Rev2mDistanceSensor sensor, DistanceUnit distanceUnit) {
        this.sensor = sensor;
        this.distanceUnit = distanceUnit;
    }

    public double getCachedDistance() {
        if(timer.milliseconds() > 25) {
            timer.reset();
            cachedValue = sensor.getDistance(distanceUnit);
        }
        return cachedValue;
    }

    public DistanceUnit getDistanceUnit() {
        return distanceUnit;
    }

    public void setDistanceUnit(DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
    }

}
