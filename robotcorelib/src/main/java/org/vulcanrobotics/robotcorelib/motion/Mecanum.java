package org.vulcanrobotics.robotcorelib.motion;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.Odometer;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import java.io.FileOutputStream;
import java.io.IOException;

public class Mecanum extends MotionProfile {

    private Odometer left, right, horizontal;
    private double radius, wheelBase, ticksPerRev, horizontalRevsPerDeg, verticalRevsPerDeg;

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
        horizontalRevsPerDeg = Double.parseDouble(properties.getProperty("rpd-horizontal"));
        verticalRevsPerDeg = Double.parseDouble(properties.getProperty("rpd-vertical"));

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

            double angle = Robot.getRobotAngleDeg();
            double encoderDifference = Math.abs(left.getPosition()) + Math.abs(right.getPosition());
            double verticalTicksPerDegree = encoderDifference / angle;
            double horizontalTicksPerDegree = horizontal.getPosition() / Math.toRadians(angle);
           properties.setProperty("rpd-horizontal", Double.toString(horizontalTicksPerDegree));
           properties.setProperty("rpd-vertical", Double.toString(verticalTicksPerDegree));
           properties.store(output, null);


            output.close();
        }
        catch(IOException e) {
            e.printStackTrace();
        }
    }

    public Odometer getLeft() {
        return left;
    }

    public void setLeft(Odometer left) {
        this.left = left;
    }

    public Odometer getRight() {
        return right;
    }

    public void setRight(Odometer right) {
        this.right = right;
    }

    public Odometer getHorizontal() {
        return horizontal;
    }

    public void setHorizontal(Odometer horizontal) {
        this.horizontal = horizontal;
    }

    public double getRadius() {
        return radius;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public double getWheelBase() {
        return wheelBase;
    }

    public void setWheelBase(double wheelBase) {
        this.wheelBase = wheelBase;
    }

    public double getTicksPerRev() {
        return ticksPerRev;
    }

    public void setTicksPerRev(double ticksPerRev) {
        this.ticksPerRev = ticksPerRev;
    }

    public double getHorizontalRevsPerDeg() {
        return horizontalRevsPerDeg;
    }

    public void setHorizontalRevsPerDeg(double horizontalRevsPerDeg) {
        this.horizontalRevsPerDeg = horizontalRevsPerDeg;
    }

   public double getVerticalRevsPerDeg() {
        return verticalRevsPerDeg;
   }

   public void setVerticalRevsPerDeg(double verticalRevsPerDeg) {
        this.verticalRevsPerDeg = verticalRevsPerDeg;
   }
}
