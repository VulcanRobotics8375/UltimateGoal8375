package org.vulcanrobotics.robotcorelib.motion;

import org.vulcanrobotics.robotcorelib.dashboard.hardware.Odometer;
import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public class Mecanum extends MotionProfile {

    private Odometer left, right, horizontal;
    private double radius, wheelBase, ticksPerRev, horizontalTicksPerDeg, countsPerCm;

    private double lastLeftPos = 0, lastRightPos = 0, lastStrafePos = 0, lastTheta = 0;

    public Mecanum(int odometerNum, Odometer... odometer) throws RobotCoreLibException {
        super(odometerNum, odometer);
    }

    public void init() throws RobotCoreLibException {
        super.init();
        if(odometerNum != 3)
            throw new RobotCoreLibException("incorrect number of odometers on motion profile");

        left = getOdometerByName("left");
        right = getOdometerByName("right");
        horizontal = getOdometerByName("strafe");

        radius = Constants.ODOMETRY_RADIUS * Constants.ODOMETRY_COUNTS_PER_CM;
        wheelBase = Constants.ODOMETRY_WHEELBASE * Constants.ODOMETRY_COUNTS_PER_CM;
        ticksPerRev = Constants.ODOMETRY_TICKS_PER_REV;
        horizontalTicksPerDeg = Constants.ODOMETRY_HORIZONTAL_TICKS_PER_REV;


    }

    public void update() {
        Point currentPos = Robot.getRobotPos();

        double leftPosition = left.getPosition();
        double rightPosition = right.getPosition();
        double horizontalPosition = horizontal.getPosition();

        double deltaLeft = ((leftPosition - lastLeftPos) / ticksPerRev) * 2.0 * Math.PI * radius;
        double deltaRight = ((rightPosition - lastRightPos) / ticksPerRev) * 2.0 * Math.PI * radius;
        double deltaHorizontalRaw = ((horizontalPosition - lastStrafePos) / ticksPerRev) * 2.0 * Math.PI * radius;
        double robotAngle = Robot.getRobotAngleRad() + (deltaLeft - deltaRight) / wheelBase;
        double deltaAngle = robotAngle - lastTheta;

        double deltaHorizontal = deltaHorizontalRaw - (Math.toDegrees(deltaAngle) * horizontalTicksPerDeg);

        currentPos.x += (((deltaLeft + deltaRight) / 2.0) * Math.sin(robotAngle)) + (deltaHorizontal * Math.cos(robotAngle));
        currentPos.y += (((deltaLeft + deltaRight) / 2.0) * Math.cos(robotAngle)) + (deltaHorizontal * Math.sin(robotAngle));

        Robot.setRobotPos(currentPos);
        Robot.setRobotAngle(robotAngle);

        lastLeftPos = leftPosition;
        lastRightPos = rightPosition;
        lastStrafePos = horizontalPosition;
        lastTheta = robotAngle;

    }

    public void calibrate(double gain) {
        while(Robot.drivetrain.getZAngle() < 90) {
            Robot.drivetrain.move(0, gain);

        }
        long lastTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - lastTime < 1000) {}

        double angle = Robot.drivetrain.getZAngle();
        double offset = Math.abs(left.getPosition()) + Math.abs(right.getPosition());
        double offsetPerDegree = offset / angle;
        double wheelBase = (2 * 90 * offsetPerDegree) / (Math.PI*(ticksPerRev));
        double horizontalTicksPerDegree = horizontal.getPosition() / Math.toRadians(angle);

        Robot.telemetry.addData("wheelBase", wheelBase);
        Robot.telemetry.addData("horizontal per deg", horizontalTicksPerDegree);
        Robot.telemetry.addData("left encoder", left.getPosition());
        Robot.telemetry.addData("right encoder", right.getPosition());
        Robot.telemetry.addData("angle", angle);

        Robot.telemetry.update();
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
        return horizontalTicksPerDeg;
    }

    public void setHorizontalRevsPerDeg(double horizontalRevsPerDeg) {
        this.horizontalTicksPerDeg = horizontalRevsPerDeg;
    }

}
