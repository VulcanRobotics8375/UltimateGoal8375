package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.vulcanrobotics.robotcorelib.framework.AutoPipeline;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.PathPoint;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.PurePursuit;
import org.vulcanrobotics.robotcorelib.robot.Robot;
import org.vulcanrobotics.robotcorelib.vision.StackDetectorCV;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;

//@Disabled
@Autonomous(name = "auto -- main", group = "example", preselectTeleOp = "main")
public class AutoPaths extends AutoPipeline {

    volatile boolean doneShooting = false;
    int sectionStart = 0;

    int wobbleGoalPosition = 2;

    @Override
    public void runOpMode() {
        try {
            //backend initialization
            autoInit();
            subsytems.drivetrain.autoInit();
            //set starting coefficients, all board measurements in CM this year
            setStart(new Point(138.34, 21.6), 0);

            //vision initialization
            StackDetectorCV detector = new StackDetectorCV();
            initVision(detector);

            //defining the data structure for path points
            ArrayList<ArrayList<PathPoint>> sections = new ArrayList<>();

            //start section-- same for each path
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
            ArrayList<PathPoint> bPosition3 = new ArrayList<>();
            ArrayList<PathPoint> bPosition4 = new ArrayList<>();
            ArrayList<PathPoint> bPositionDum = new ArrayList<>();

            //4 ring auto
            ArrayList<PathPoint> cPosition1 = new ArrayList<>();
            ArrayList<PathPoint> cPosition2 = new ArrayList<>();
            ArrayList<PathPoint> cPositionDum = new ArrayList<>();
            ArrayList<PathPoint> cPosition3 = new ArrayList<>();
            ArrayList<PathPoint> cPosition4 = new ArrayList<>();

            start.add(new PathPoint(138.34, 21.6, -0.05, 1, 20, 0));
            start.add(new PathPoint(96, 80, -0.05, 1, 20, 0));
            start.add(new PathPoint(105, 155, -0.025, 1, 20, 0));

            aPosition1.add(new PathPoint(188, 225, -0.03, 1, 20, 0));

            aPosition2.add(new PathPoint(192, 150, -0.025, 1, 20, 0));
            aPosition3.add(new PathPoint(192, 150, -0.025, 1, 20, 0));
            aPosition3.add(new PathPoint(191, 100, -0.025, 1, 20, Math.PI / 2.0));
            aPosition3.add(new PathPoint(173, 102, -0.025, 1, 20, Math.PI / 2.0));

            aPosition4.add(new PathPoint(173, 102, -0.025, 1, 20, Math.PI / 2.0));
            aPosition4.add(new PathPoint(188, 100, -0.025, 1, 20, Math.PI / 2.0));
            aPosition4.add(new PathPoint(172, 210, -0.025, 1, 20, 0));
            aPosition4.add(new PathPoint(171, 210, -0.025, 1, 20, 0));

            bPosition1.add(new PathPoint(128, 273, -0.025, 1, 20, 0));

            bPosition2.add(new PathPoint(128, 273, -0.02, 1, 20, 0));
            bPosition2.add(new PathPoint(97, 81, -0.03, 1, 20, 0));
            bPosition2.add(new PathPoint(142, 60, -0.03, 1, 20, 0));

            bPositionDum.add(new PathPoint(142, 60, -0.025, 1, 20, 0));

            bPosition3.add(new PathPoint(142, 60, -0.025, 1, 20, 0));
            bPosition3.add(new PathPoint(141, 170, -0.03, 1, 20, 0));
            bPosition3.add(new PathPoint(143, 170, -0.03, 1, 20, 0));

            bPosition4.add(new PathPoint(142, 170, -0.03, 1, 20, 0));
            bPosition4.add(new PathPoint(120, 265, -0.025, 1, 20, 0));
            bPosition4.add(new PathPoint(121, 265, -0.03, 1, 20, 0));

            cPosition1.add(new PathPoint(190, 310, -0.04, 1, 20, 0));
//            cPosition1.add(new PathPoint(189, 309, -0.025, 1, 20, 0));

//            cPosition2.add(new PathPoint(155, 305, -0.03, 1, 20, 0));
            cPosition2.add(new PathPoint(118, 273, -0.05, 1, 20, Math.PI / 2.0));
            cPosition2.add(new PathPoint(83, 81, -0.04, 1, 20, Math.PI - 0.01));
            cPosition2.add(new PathPoint(142, 60, -0.03, 1, 20, 0));

            cPositionDum.add(new PathPoint(142, 60, -0.03, 1, 20, 0));

            cPosition3.add(new PathPoint(142, 60, -0.05, 1, 20, 0));
            cPosition3.add(new PathPoint(141, 130, -0.04, 1, 20, 0));
            cPosition3.add(new PathPoint(143, 130, -0.04, 1, 20, 0));

            cPosition4.add(new PathPoint(142, 130, -0.04, 1, 20, 0));
            cPosition4.add(new PathPoint(177, 330, -0.04, 1, 20, 0));

            park.add(new PathPoint(115, 210, -0.025, 1, 20, 0));


            //initialize the Pipeline Controller with whatever controller we are using
            controller = new PurePursuit(sections);

            //internal controller for using
           final PurePursuit internalController = (PurePursuit) controller;

           telemetry.addLine("ready");
           telemetry.update();

            waitForStart();

            /*
            VISION
             */

            wobbleGoalPosition = detector.getStackHeight();

            if(wobbleGoalPosition == 0) {
                sections.add(start);
                sections.add(aPosition1);
                sections.add(aPosition2);
                sections.add(aPosition3);
                sections.add(aPosition4);
                sections.add(park);
            }
            else if(wobbleGoalPosition == 1) {
                sections.add(start);
                sections.add(bPosition1);
                sections.add(bPosition2);
                sections.add(bPositionDum);
                sections.add(bPosition3);
                sections.add(bPosition4);
                sections.add(park);
            }
            else if(wobbleGoalPosition == 2) {
                sections.add(start);
                sections.add(cPosition1);
                sections.add(cPosition2);
                sections.add(cPositionDum);
                sections.add(cPosition3);
                sections.add(cPosition4);
                sections.add(park);
            }

            //interrupt handler makes sure the robot stops when the stop button is pressed.
            Robot.startOdometryThread();
            startInterruptHandler();

            new Thread(new Runnable() {
                @Override
                public void run() {
                    sleep(1500);
                    if(!isStopRequested()) {
                        controller.run();
                    }
                }
            }).start();

            boolean secondWobble = false;
            subsytems.drivetrain.setAngleOffset(0);
//            subsytems.shooter.setPIDFCoefficients(new PIDFCoefficients(10.0, 3.0, 0.0, 0.0));
            subsytems.drivetrain.setAngleOffset(3);
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
                    int ringCount = 0;

                    long lastTime = System.currentTimeMillis();

                    while(System.currentTimeMillis() - lastTime < 2000) {
                        subsytems.drivetrain.mecanumDrive(0, 0, 0, true, false, false, false, false, false);
                        subsytems.shooter.setShooterPower(0.83);
                    }

                    lastTime = System.currentTimeMillis();

                    while(ringCount == 0 && !isStopRequested()) {
                        subsytems.drivetrain.mecanumDrive(0, 0, 0, true, false, false, false, false, false);
                        subsytems.shooter.setHopperPosition(0.35);
                        if(System.currentTimeMillis() - lastTime < 750) {
                            continue;
                        }
                        ringCount++;
                    }
                    lastTime = System.currentTimeMillis();
//                    subsytems.shooter.setHopperPosition(0.0);

                    while(ringCount == 1 && !isStopRequested()) {
                        subsytems.shooter.setHopperPosition(0.0);
                        if(System.currentTimeMillis() - lastTime < 750) {
                            continue;
                        }
                        ringCount++;

                    }

                    lastTime = System.currentTimeMillis();
                    while(ringCount == 2 && !isStopRequested()) {
                        subsytems.shooter.setHopperPosition(0.35);
                        if(System.currentTimeMillis() - lastTime < 750) {
                            continue;
                        }
                        ringCount++;

                    }
                    lastTime = System.currentTimeMillis();
                    while(ringCount == 3 && !isStopRequested()) {
                        subsytems.shooter.setHopperPosition(0.0);
                        if(System.currentTimeMillis() - lastTime < 750) {
                            continue;
                        }
                        ringCount++;

                    }
                    lastTime = System.currentTimeMillis();
                    while(ringCount == 4 && !isStopRequested()) {
                        subsytems.shooter.setHopperPosition(0.35);
                        if(System.currentTimeMillis() - lastTime < 750) {
                            continue;
                        }
                        ringCount++;

                    }
                    subsytems.shooter.setHopperPosition(0.0);
                    doneShooting = true;
                    if(sectionStart == 0) {
                        internalController.startNextSection();
                        sectionStart++;
                    }
                }
                subsytems.shooter.setShooterPower(0);

                /*
                CASE 0
                 */
                if (wobbleGoalPosition == 0) {
                    if (internalController.getCurrentSection() == 2) {
                        subsytems.wobbleGrabber.setGrabPosition(1.0);
                        subsytems.wobbleGrabber.setTurnPosition(0);
                        sleep(500);
                        if (sectionStart == 1) {
                            internalController.startNextSection();
                            sectionStart++;
                        }
                    }
                    if (internalController.getCurrentSection() == 3) {
                        if (sectionStart == 2) {
                            internalController.startNextSection();
                            sectionStart++;
                        }
                    }

                    if (internalController.getCurrentSection() == 4) {
                        subsytems.wobbleGrabber.setTurnPosition(0.2);
                        subsytems.wobbleGrabber.setGrabPosition(0.05);
//                            subsytems.intake.run(true, false, false);
                        sleep(1000);
                        if (sectionStart == 3) {
                            internalController.startNextSection();
                            sectionStart++;
                        }
                    }

                    if (internalController.getCurrentSection() == 5) {
//                            subsytems.shooter.setShooterPower(0.82);
                        subsytems.wobbleGrabber.setGrabPosition(0.2);
//                            subsytems.wobbleGrabber.setTurnPosition(0.0);
                        if (sectionStart == 4) {
                            internalController.startNextSection();
                            sectionStart++;
                        }

                    }

                    if (internalController.getCurrentSection() == 6) {
                        subsytems.wobbleGrabber.setGrabPosition(1.0);
                        subsytems.wobbleGrabber.setTurnPosition(0.0);
                        if (sectionStart == 5) {
                            internalController.startNextSection();
                            sectionStart++;
                        }
                    }
                }

                    /*
                     * CASE 1
                     */
                else if(wobbleGoalPosition == 1) {
                    if (internalController.getCurrentSection() == 2) {
                        subsytems.wobbleGrabber.setGrabPosition(1.0);
                        subsytems.wobbleGrabber.setTurnPosition(0);
                        sleep(500);
                        if (sectionStart == 1) {
                            internalController.startNextSection();
                            sectionStart++;
                        }
                    }
                    if (internalController.getCurrentSection() == 3) {
                        if (sectionStart == 2) {
                            internalController.startNextSection();
                            sectionStart++;
                        }
                    }

                    if (internalController.getCurrentSection() == 4) {
                        subsytems.wobbleGrabber.setTurnPosition(0.2);
                        subsytems.wobbleGrabber.setGrabPosition(0.05);
                        subsytems.intake.run(true, false, false);
                        sleep(1000);
                        if (sectionStart == 3) {
                            internalController.startNextSection();
                            sectionStart++;
                        }
                    }

                    if (internalController.getCurrentSection() == 5 && sectionStart == 4) {
                        long lastTime = System.currentTimeMillis();
                        subsytems.intake.run(false, false, false);
                        subsytems.shooter.setShooterPower(0.82);
                       while(System.currentTimeMillis() - lastTime < 1500) {
                           subsytems.drivetrain.mecanumDrive(0, 0, 0, true, false, false,false, false, false);
                       }
                        subsytems.shooter.setHopperPosition(0.35);
                        sleep(500);
                        if (sectionStart == 4) {
                            internalController.startNextSection();
                            sectionStart++;
                        }
                    }
                    if (internalController.getCurrentSection() == 6) {
                        subsytems.wobbleGrabber.setGrabPosition(1.0);
                        subsytems.wobbleGrabber.setTurnPosition(0.0);
                        sleep(1000);
                        if (sectionStart == 5) {
                            internalController.startNextSection();
                        }
                    }
                }

                /*
                CASE 2
                 */
                else if(wobbleGoalPosition == 2) {
                    if (internalController.getCurrentSection() == 2) {
                        subsytems.wobbleGrabber.setGrabPosition(1.0);
                        subsytems.wobbleGrabber.setTurnPosition(0);
                        sleep(500);
                        if (sectionStart == 1) {
                            internalController.startNextSection();
                            sectionStart++;
                        }
                    }
                    if (internalController.getCurrentSection() == 3) {
                        if (sectionStart == 2) {
                            internalController.startNextSection();
                            sectionStart++;
                        }
                    }

                    if (internalController.getCurrentSection() == 4) {
                        subsytems.wobbleGrabber.setTurnPosition(0.2);
                        subsytems.wobbleGrabber.setGrabPosition(0.05);
                        subsytems.intake.run(true, false, false);
                        sleep(1000);
                        if (sectionStart == 3) {
                            internalController.startNextSection();
                            sectionStart++;
                        }
                    }

                    if (internalController.getCurrentSection() == 5 && sectionStart == 4) {
                        subsytems.drivetrain.setAngleOffset(-3);
                        long lastTime = System.currentTimeMillis();
                        subsytems.intake.run(true, false, false);
                        subsytems.shooter.setShooterPower(0.82);
                        while(System.currentTimeMillis() - lastTime < 1500) {
                            subsytems.drivetrain.mecanumDrive(0, 0, 0, true, false, false,false, false, false);
                        }
                        subsytems.intake.run(false, false, false);
                        subsytems.shooter.setHopperPosition(0.35);
                        sleep(500);
                        if (sectionStart == 4) {
                            internalController.startNextSection();
                            sectionStart++;
                        }
                    }
                    if (internalController.getCurrentSection() == 6) {
                        subsytems.wobbleGrabber.setGrabPosition(1.0);
                        subsytems.wobbleGrabber.setTurnPosition(0.0);
                        sleep(500);
                        if (sectionStart == 5) {
                            internalController.startNextSection();
                        }
                    }

                }

                if(isStopRequested())
                    break;
            }

            Robot.storeRobotPosition();
            controller.stop = true;

            //should be a fix to the jittering issue at an early shut down event
            subsytems.drivetrain.setPowers(0, 0, 0, 0);

        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }




    }
}
