package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.vulcanrobotics.robotcorelib.framework.AutoPipeline;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.PathPoint;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.PurePursuit;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;

//@Disabled
@Autonomous(name = "example path", group = "example")
public class ExamplePath extends AutoPipeline {

    volatile boolean doneShooting = false;

    @Override
    public void runOpMode() {
        try {
            //backend initialization
            autoInit();
            //set starting coefficients, all board measurements in CM this year
            setStart(new Point(138.34, 21.6), 0);
            //defining the data structure for path points
            ArrayList<ArrayList<PathPoint>> sections = new ArrayList<>();
            ArrayList<PathPoint> section1 = new ArrayList<>();
            ArrayList<PathPoint> section2 = new ArrayList<>();
            ArrayList<PathPoint> section3 = new ArrayList<>();

            section1.add(new PathPoint(138.34, 21.6, -0.05, 1, 20, 0));
            section1.add(new PathPoint(100, 80, -0.05, 1, 20, 0));
            section1.add(new PathPoint(110, 145, -0.05, 1, 20, 0));

            section2.add(new PathPoint(185, 223, -0.025, 1, 20, 0));
//            section2.add(new PathPoint(135, 35, -0.05, 1, 20, 0));

            sections.add(section1);
            sections.add(section2);
//            sections.add(section3);

            //path points here

            //initialize the Pipeline Controller with whatever controller we are using
            controller = new PurePursuit(sections);

           final PurePursuit internalController = (PurePursuit) controller;

           telemetry.addLine("ready");
           telemetry.update();

            waitForStart();

            //interrupt handler makes sure the robot stops when the stop button is pressed.
            Robot.startOdometryThread();
            startInterruptHandler();

            new Thread(new Runnable() {
                @Override
                public void run() {
                    sleep(1500);
                    controller.run();
                }
            }).start();

            while(opModeIsActive()) {

                PathPoint currentPoint = internalController.getCurrentPoint();
                //subsystem code here, triggers are currentPoint and then timing
//                internalController.moveToPoint(section1.get(1));

                telemetry.addData("robot x", Robot.getRobotX());
                telemetry.addData("robot y", Robot.getRobotY());
                telemetry.addData("robot angle", Robot.getRobotAngleDeg());
                telemetry.addData("current section", internalController.getCurrentSection());

                telemetry.update();
//                subsytems.wobbleGrabber.run(false, true, 0);
                if(internalController.getCurrentSection() < 2) {
                    subsytems.wobbleGrabber.setGrabPosition(0.05);
                    subsytems.wobbleGrabber.setTurnPosition(0.2);

                } else {
                    subsytems.wobbleGrabber.setGrabPosition(0.7);
                }

                if(internalController.getCurrentSection() == 1 && !doneShooting) {
//                    internalController.startNextSection();
                    int ringCount = 0;
                    boolean ringOneShot = false, ringTwoShot = false, ringThreeShot = false;
                    long lastTime = System.currentTimeMillis();
                    subsytems.shooter.setHopperPosition(0);
                    subsytems.shooter.setShooterPower(0.74);
                    while(ringCount == 0 && !isStopRequested()) {
                        subsytems.drivetrain.mecanumDrive(0, 0, 0, false, false, false, true, false, false);
                        if(System.currentTimeMillis() - lastTime < 1650 && !ringOneShot) {
                            continue;
                        }
                        if(!ringOneShot) {
                            lastTime = System.currentTimeMillis();
                            ringOneShot = true;
                        }
                        subsytems.shooter.setHopperPosition(0.35);
                        if(System.currentTimeMillis() - lastTime < 500) {
                            subsytems.shooter.setHopperPosition(0.35);
                            continue;
                        }
//                        subsytems.shooter.setHopperPosition(0);
                        ringCount++;

                    }
                    subsytems.shooter.setHopperPosition(0);
                    lastTime = System.currentTimeMillis();
                    subsytems.drivetrain.setAngleOffset(-1);
                    while(ringCount == 1 && !isStopRequested()) {
                        subsytems.drivetrain.mecanumDrive(0, 0, 0, false, false, true, false, false, false);
                        if(System.currentTimeMillis() - lastTime < 750 && !ringTwoShot) {
//                            subsytems.shooter.setHopperPosition(0);
                            continue;
                        }
                        if(!ringTwoShot) {
                            lastTime = System.currentTimeMillis();
                            ringTwoShot = true;
                        }
                        subsytems.shooter.setHopperPosition(0.35);
                        if(System.currentTimeMillis() - lastTime < 500) {
                            subsytems.shooter.setHopperPosition(0.35);
                            continue;
                        }
                        subsytems.shooter.setHopperPosition(0);
                        ringCount++;
                    }

                    lastTime = System.currentTimeMillis();
                    while (ringCount == 2 && !isStopRequested()) {
                        subsytems.drivetrain.mecanumDrive(0, 0, 0, false, true, false, false, false, false);
                        if(System.currentTimeMillis() - lastTime < 750 && !ringThreeShot) {
                            continue;
                        }
                        if(!ringThreeShot) {
                            lastTime = System.currentTimeMillis();
                            ringThreeShot = true;
                        }
                        subsytems.shooter.setHopperPosition(0.35);
                        if(System.currentTimeMillis() - lastTime < 500) {
                            continue;
                        }
                        subsytems.shooter.setHopperPosition(0);
                        ringCount++;
                    }
                        doneShooting = true;
                    internalController.startNextSection();
                }
                subsytems.shooter.setShooterPower(0);

                if (internalController.getCurrentSection() == 2) {

                }

                if(isStopRequested())
                    break;
            }

            Robot.storeRobotPosition();
            controller.stop = true;

        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }




    }
}
