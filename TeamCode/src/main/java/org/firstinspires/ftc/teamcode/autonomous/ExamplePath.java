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
@Autonomous(name = "auto -- main", group = "example")
public class ExamplePath extends AutoPipeline {

    volatile boolean doneShooting = false;
    int sectionStart = 0;

    @Override
    public void runOpMode() {
        try {
            //backend initialization
            autoInit();
            subsytems.drivetrain.autoInit();
            //set starting coefficients, all board measurements in CM this year
            setStart(new Point(138.34, 21.6), 0);
            //defining the data structure for path points
            ArrayList<ArrayList<PathPoint>> sections = new ArrayList<>();

            //powershot section-- same for each path
            ArrayList<PathPoint> start = new ArrayList<>();

            //0 ring auto
            ArrayList<PathPoint> aPosition1 = new ArrayList<>();
            ArrayList<PathPoint> aPosition2 = new ArrayList<>();
            ArrayList<PathPoint> aPosition3 = new ArrayList<>();
            ArrayList<PathPoint> aPosition4 = new ArrayList<>();
            ArrayList<PathPoint> park = new ArrayList<>();

            //1 ring auto
            ArrayList<PathPoint> bPosition1 = new ArrayList<>();
            ArrayList<PathPoint> bPosition2 = new ArrayList<>();

            //4 ring auto
            ArrayList<PathPoint> cPosition1 = new ArrayList<>();

            start.add(new PathPoint(138.34, 21.6, -0.05, 1, 20, 0));
            start.add(new PathPoint(96, 80, -0.05, 1, 20, 0));
            start.add(new PathPoint(105, 155, -0.025, 1, 20, 0));

            aPosition1.add(new PathPoint(188, 225, -0.025, 1, 20, 0));

            aPosition2.add(new PathPoint(192, 150, -0.025, 1, 20, 0));
            aPosition3.add(new PathPoint(192, 150, -0.025, 1, 20, 0));
            aPosition3.add(new PathPoint(191, 100, -0.025, 1, 20, Math.PI / 2.0));
            aPosition3.add(new PathPoint(173, 102, -0.025, 1, 20, Math.PI / 2.0));

            aPosition4.add(new PathPoint(173, 102, -0.025, 1, 20, Math.PI / 2.0));
            aPosition4.add(new PathPoint(188, 100, -0.025, 1, 20, Math.PI / 2.0));
            aPosition4.add(new PathPoint(172, 210, -0.025, 1, 20, 0));
            aPosition4.add(new PathPoint(171, 211, -0.025, 1, 20, 0));


            bPosition1.add(new PathPoint(132, 273, -0.025, 1, 20, 0));

            bPosition2.add(new PathPoint(173, 102, -0.025, 1, 20, Math.PI / 2.0));
            bPosition2.add(new PathPoint(188, 100, -0.025, 1, 20, Math.PI / 2.0));
            bPosition2.add(new PathPoint(105, 270, -0.05, 1, 20, 0));
            bPosition2.add(new PathPoint(97, 265, -0.025, 1, 20, 0));

            park.add(new PathPoint(115, 210, -0.025, 1, 20, 0));

            sections.add(start);
            sections.add(bPosition1);
            sections.add(aPosition2);
            sections.add(aPosition3);
            sections.add(bPosition2);
            sections.add(park);

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

            boolean secondWobble = false;
            while(opModeIsActive()) {

                PathPoint currentPoint = internalController.getCurrentPoint();
                //subsystem code here, triggers are currentPoint and then timing
//                internalController.moveToPoint(section1.get(1));

                telemetry.addData("robot x", Robot.getRobotX());
                telemetry.addData("robot y", Robot.getRobotY());
                telemetry.addData("robot angle", Robot.getRobotAngleDeg());
                telemetry.addData("current section", internalController.getCurrentSection());
                telemetry.addData("section start", sectionStart);

                telemetry.update();
//                subsytems.wobbleGrabber.run(false, true, 0);
                if(internalController.getCurrentSection() < 2) {
                    subsytems.wobbleGrabber.setGrabPosition(0.05);
                    subsytems.wobbleGrabber.setTurnPosition(0.2);

                }

                if(internalController.getCurrentSection() == 1 && !doneShooting) {
//                    internalController.startNextSection();
                    int ringCount = 0;
                    boolean ringOneShot = false, ringTwoShot = false, ringThreeShot = false;
                    long lastTime = System.currentTimeMillis();
                    subsytems.shooter.setHopperPosition(0);
                    subsytems.shooter.setShooterPower(0.75);
                    subsytems.drivetrain.setAngleOffset(1);
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
                        if(System.currentTimeMillis() - lastTime < 1750 && !ringTwoShot) {
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
                        if(System.currentTimeMillis() - lastTime < 1750 && !ringThreeShot) {
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
                    if(sectionStart == 0) {
                        internalController.startNextSection();
                        sectionStart++;
                    }

                }
                subsytems.shooter.setShooterPower(0);

                if (internalController.getCurrentSection() == 2) {
                    subsytems.wobbleGrabber.setGrabPosition(1.0);
                    subsytems.wobbleGrabber.setTurnPosition(0);
                    sleep(500);
                    if(sectionStart == 1) {
                        internalController.startNextSection();
                        sectionStart++;
                    }
                }
                if(internalController.getCurrentSection() == 3) {
                    if(sectionStart == 2) {
                        internalController.startNextSection();
                        sectionStart++;
                    }
                }

                if(internalController.getCurrentSection() == 4) {
                    subsytems.wobbleGrabber.setTurnPosition(0.2);
                    subsytems.wobbleGrabber.setGrabPosition(0.05);
                    sleep(1000);
                    if(sectionStart == 3) {
                        internalController.startNextSection();
                        sectionStart++;
                    }
                }
                if(internalController.getCurrentSection() == 5) {
                    subsytems.wobbleGrabber.setGrabPosition(1.0);
                    subsytems.wobbleGrabber.setTurnPosition(0.0);
                    if(sectionStart == 4) {
                        internalController.startNextSection();
                        sectionStart++;
                    }
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
