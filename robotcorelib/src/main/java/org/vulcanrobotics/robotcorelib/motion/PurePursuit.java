package org.vulcanrobotics.robotcorelib.motion;

import org.vulcanrobotics.robotcorelib.math.Functions;
import org.vulcanrobotics.robotcorelib.math.PID;
import org.vulcanrobotics.robotcorelib.math.PathPoint;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import java.util.ArrayList;

public class PurePursuit extends Controller {

    ArrayList<ArrayList<PathPoint>> sections;

    private PID turnPID = new PID(1.8, 0, 1);
    private volatile int currentSection = 0;
    private volatile boolean start = false;

    public PurePursuit(ArrayList<ArrayList<PathPoint>> sections) {
        this.sections = sections;
    }

    public void run() {
        int currentSection = 0;
        for (ArrayList<PathPoint> section : sections) {
            start = false;
            while(Math.hypot(section.get(section.size() - 1).x - Robot.getRobotX(), section.get(section.size() - 1).y - Robot.getRobotY()) > 6) {
                PathPoint followPoint = findFollowPoint(section);
                moveToPoint(followPoint);
                if(stop)
                    break;
            }
            currentSection++;
            this.currentSection = currentSection;
            Robot.drivetrain.setPowers(0, 0, 0, 0);
            waitForStart();

        }
        Robot.drivetrain.setPowers(0, 0, 0, 0);

    }

    private PathPoint findFollowPoint(ArrayList<PathPoint> path) {
        PathPoint followPoint = path.get(0);

        ArrayList<Point> circleIntersections;

        for (int i = 0; i < path.size() - 1; i++) {
            PathPoint start = path.get(i);
            PathPoint end = path.get(i + 1);

            if(path.indexOf(end) == path.size() - 1) {
                circleIntersections = Functions.lineCircleIntersectNoBoundingBox(start.toPoint(), end.toPoint(), end.lookAhead, Robot.getRobotPos());
            } else {
                circleIntersections = Functions.lineCircleIntersect(start.toPoint(), end.toPoint(), end.lookAhead, Robot.getRobotPos());
            }

            double closestAngle = Double.MAX_VALUE;
            for (Point intersection : circleIntersections) {
                double angle = Math.atan2(intersection.y - Robot.getRobotX(), intersection.x - Robot.getRobotY());
                double relativePointAngle = Math.atan2(end.y - start.y, end.x - start.x);
                double deltaAngle = Math.abs(Functions.angleWrap(angle - relativePointAngle));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followPoint.setPathPoint(end);
                    followPoint.setPoint(intersection);
                }

                //new, might not work
                if(path.indexOf(end) == path.size() - 1) {
                    double maxX = Math.max(start.x, end.x);
                    if(followPoint.x > maxX) {
                        followPoint.setPoint(end.toPoint());
                    }
                }

            }
        }

        currentPoint.setPathPoint(followPoint);

        return followPoint;
    }

    public void moveToPoint(PathPoint point) {
        double absoluteAngleToPoint = Math.atan2(point.y - Robot.getRobotY(), point.x - Robot.getRobotX());

        double robotAngleToPoint = Robot.getRobotAngleRad();

        turnPID.run(point.angle, robotAngleToPoint);

        double turnSpeed = turnPID.getOutput() * point.turnSpeed;

        double distanceToPoint = Math.hypot(point.x - Robot.getRobotX(), point.y - Robot.getRobotY()) * point.speed;

        Robot.drivetrain.fieldCentricMove(Math.cos(absoluteAngleToPoint) * distanceToPoint, Math.sin(absoluteAngleToPoint) * distanceToPoint, turnSpeed);
    }

    public int getCurrentSection(){
        return currentSection;
    }

    private void waitForStart() {
        while(!start) {
            if(stop) {
                break;
            }
        }
    }

    public void startNextSection() {
        start = true;
    }

}
