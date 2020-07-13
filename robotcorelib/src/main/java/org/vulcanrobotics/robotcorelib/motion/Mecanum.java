package org.vulcanrobotics.robotcorelib.motion;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.vulcanrobotics.robotcorelib.Dashboard.Hardware.Odometer;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import java.io.FileOutputStream;
import java.io.IOException;

public class Mecanum extends MotionProfile {

    private Odometer left, right, horizontal;
    private double radius, wheelBase, ticksPerRev, revsPerDeg, driftTolerance;

    public Mecanum(int odometerNum, Odometer... odometer) throws RobotCoreLibException {
        super(odometerNum, odometer);
    }

    public void init() throws RobotCoreLibException {
        super.init();
        if(odometerNum != 3)
            throw new RobotCoreLibException("incorrect number of odometers on motion profile");

        left = getOdometerByName("left");
        right = getOdometerByName("right");
        horizontal = getOdometerByName("horizontal");

        radius = Double.parseDouble(properties.getProperty("radius"));
        wheelBase = Double.parseDouble(properties.getProperty("wheelBase"));
        ticksPerRev = Double.parseDouble(properties.getProperty("tpr"));
        revsPerDeg = Double.parseDouble(properties.getProperty("rpd"));
        driftTolerance = Double.parseDouble(properties.getProperty("driftTolerance"));

    }

    public void update() {

    }

    public void calibrate(double gain) {
        while(Robot.getRobotAngleDeg() < 90) {
            Robot.drivetrain.move(0, gain);
        }

        try {
            String path = AppUtil.getInstance().getSettingsFile("robotconfig.properties").getAbsolutePath();

            //set up file streams for writing to the properties file
            FileOutputStream output = new FileOutputStream(path);

           double distanceTravelledLeft = (left.getPosition() / ticksPerRev) * radius;
           double distanceTravelledRight = (right.getPosition() / ticksPerRev) * radius;

           if(Math.abs(Math.abs(distanceTravelledLeft) - Math.abs(distanceTravelledRight)) > driftTolerance)
               throw new RobotCoreLibException("calibration error-- too much variation between encoders");

           wheelBase = distanceTravelledLeft / Math.toRadians(90);
           revsPerDeg = (horizontal.getPosition() / ticksPerRev) / 90;

           properties.setProperty("wheelBase", String.valueOf(wheelBase));
           properties.setProperty("rpd", String.valueOf(revsPerDeg));
           properties.store(output, null);

           output.close();
        }
        catch(IOException | RobotCoreLibException e) {
            e.printStackTrace();
        }

    }
}
