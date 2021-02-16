package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.vulcanrobotics.robotcorelib.framework.AutoPipeline;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.PathPoint;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.PurePursuit;
import org.vulcanrobotics.robotcorelib.robot.Robot;
import org.vulcanrobotics.robotcorelib.vision.StackDetectorCV;

import java.util.ArrayList;

@Autonomous(name = "new auto", group = "main", preselectTeleOp = "main")
public class DalyCityAuto extends AutoPipeline {
    @Override
    public void runOpMode() {
        try {
            autoInit();
        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }
        subsystems.drivetrain.autoInit();
        StackDetectorCV stackDetector = new StackDetectorCV();
        initVision(stackDetector);
        subsystems.wobbleGrabber.wobbleGrab.setPosition(0.05);

        int path = 2;

        setStart(new Point(138.34, 21.6), 0.0);

        ArrayList<ArrayList<PathPoint>> sections = new ArrayList<>();

        //pathPoint init here
        ArrayList<PathPoint> start1 = new ArrayList<>();
        start1.add(new PathPoint(138.34, 21.6, -1, 1, 12, 0));
        start1.add(new PathPoint(138.34, 21.6, -1, 1, 12, 0));
        start1.add(new PathPoint(95, 80, -0.5, 1, 12, 0));
//        start1.add(new PathPoint(100, 170, -0.5, 1, 12, 0));

        ArrayList<PathPoint> start2 = new ArrayList<>();
        start2.add(new PathPoint(100, 170, -0.5, 1, 12, 0));

        ArrayList<PathPoint> section1 = new ArrayList<>();
        section1.add(new PathPoint(100, 170, -1, 1, 12, 0));
        section1.add(new PathPoint(185, 180, -1, 1, 15, 0));

        ArrayList<PathPoint> section2 = new ArrayList<>();
//        section2.add(new PathPoint(185, 180, -1, 1, 15, 0));
        section2.add(new PathPoint(185, 150, -1, 1, 15, 0));
        section2.add(new PathPoint(185, 150, -1, 1, 15, 0));
        section2.add(new PathPoint(210, 100, -0.5, 1, 15, Math.PI));

        ArrayList<PathPoint> section3 = new ArrayList<>();
        section3.add(new PathPoint(175, 155, -0.7, 1, 12, 2.0 * Math.PI));

        ArrayList<PathPoint> section4 = new ArrayList<>();
        section4.add(new PathPoint(100, 195, -1, 1, 12, 2.0 * Math.PI));


       //case 0
//        sections.add(start1);
//        sections.add(start2);
//        sections.add(section1);
//        sections.add(section2);
//        sections.add(section3);
//        sections.add(section4);

        ArrayList<PathPoint> sectionB1 = new ArrayList<>();
        sectionB1.add(new PathPoint(100, 170, -0.5, 1, 12, 0));
        sectionB1.add(new PathPoint(135, 230, -1, 1, 12, 0));

        ArrayList<PathPoint> sectionB2 = new ArrayList<>();
//        sectionB2.add(new PathPoint(135, 210, -1, 1, 15, Math.PI));
        sectionB2.add(new PathPoint(136, 200, -1, 1, 12, 0));
        sectionB2.add(new PathPoint(160, 135, -0.5, 0.5, 15, Math.PI));

        ArrayList<PathPoint> sectionB3 = new ArrayList<>();
        sectionB3.add(new PathPoint(160, 135, -0.5, 0.5, 15, Math.PI));
        sectionB3.add(new PathPoint(200, 130, -0.5, 1, 15, Math.PI));
        sectionB3.add(new PathPoint(201, 105, -0.5, 1, 15, Math.PI));

        ArrayList<PathPoint> sectionB4 = new ArrayList<>();
        sectionB4.add(new PathPoint(135, 160, -1, 1, 12, 2.0 * Math.PI));

        ArrayList<PathPoint> sectionB5 = new ArrayList<>();
        sectionB5.add(new PathPoint(130, 205, -1, 1, 15, 2.0 * Math.PI));

        ArrayList<PathPoint> sectionB6 = new ArrayList<>();
        sectionB6.add(new PathPoint(100, 210, -1, 1, 15, 2.0 * Math.PI));

        ArrayList<PathPoint> sectionC1 = new ArrayList<>();
        sectionC1.add(new PathPoint(100, 170, -0.5, 1, 12, 0));
        sectionC1.add(new PathPoint(175, 300, -1, 1, 12, 0));

        ArrayList<PathPoint> sectionC2 = new ArrayList<>();
        sectionC2.add(new PathPoint(136, 200, -1, 1, 12, 0));
        sectionC2.add(new PathPoint(160, 135, -1, 0.8, 15, Math.PI));

        ArrayList<PathPoint> sectionCTest = new ArrayList<>();
        sectionCTest.add(new PathPoint(175, 300, -1, 1, 12, 0));
        sectionCTest.add(new PathPoint(135, 200, -1, 1, 12, 0));

        ArrayList<PathPoint> sectionC3 = new ArrayList<>();
        sectionC3.add(new PathPoint(160, 135, -0.5, 1, 6, Math.PI));
        sectionC3.add(new PathPoint(190, 132, -0.5, 1, 6, Math.PI));
        sectionC3.add(new PathPoint(206, 110, -0.5, 1, 6, Math.PI));

        ArrayList<PathPoint> sectionC4 = new ArrayList<>();
        sectionC4.add(new PathPoint(170, 280, -1, 1, 12, 2.0 * Math.PI));




        telemetry.addLine("ready");
        telemetry.update();
        waitForStart();

        //assign path number based on stack detection
        path = stackDetector.getStackHeight();

        if(path == 0) {
            sections.add(start1);
            sections.add(start2);
            sections.add(section1);
            sections.add(section2);
            sections.add(section3);
            sections.add(section4);
        } else if(path == 1) {
            sections.add(start1);
            sections.add(start2);
            sections.add(sectionB1);
            sections.add(sectionB2);
            sections.add(sectionB3);
            sections.add(sectionB4);
            sections.add(sectionB5);
            sections.add(sectionB6);
        } else if (path == 2) {
            sections.add(start1);
            sections.add(start2);
            sections.add(sectionC1);
            sections.add(sectionCTest);
            sections.add(sectionC2);
            sections.add(sectionC3);
            sections.add(sectionB4);
            sections.add(sectionC4);
            sections.add(sectionB6);
        }

        final PurePursuit controller = new PurePursuit(sections);

        super.controller = controller;

        Robot.startOdometryThread();
        startInterruptHandler();

        new Thread(new Runnable() {
            @Override
            public void run() {
                //potential sleep here
                if(!isStopRequested()) {
                    controller.run();
                }
            }
        }).start();

        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()) {
                    telemetry.addData("x", Robot.getRobotX());
                    telemetry.addData("y", Robot.getRobotY());
                    telemetry.addData("theta", Robot.getRobotAngleDeg());
                    telemetry.update();
                }
            }
        }).start();

        //im stupid,,, just remove the while(opModeIsActive()) and everything is better
        subsystems.drivetrain.resetAiming();
        while(controller.getCurrentSection() == 0 && !isStopRequested()) {
            subsystems.shooter.setPowers(0.82);
//            subsystems.shooter.setPowerShotPosition(0.035);
            subsystems.wobbleGrabber.wobbleGrab.setPosition(0.05);
            subsystems.wobbleGrabber.wobbleTurn.setPosition(0.52);
            //any point specific actions can start here

        }
        controller.startNextSection();
        while(controller.getCurrentSection() == 1) {}
        sleep(50);
        subsystems.shooter.setPowers(0.82);
        while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
            subsystems.shooter.setHopperPosition(0.0);
            subsystems.shooter.setPowers(0.82);
            subsystems.drivetrain.aim(0, 0.12,0.009);
        }
        subsystems.drivetrain.run(0, 0);
        sleep(100);
        subsystems.shooter.setHopperPosition(0.2);
        sleep(200);
        subsystems.shooter.setHopperPosition(0.0);
        sleep(200);
        subsystems.shooter.setHopperPosition(0.2);
        sleep(200);
        subsystems.shooter.setHopperPosition(0.0);
        sleep(200);
        subsystems.shooter.setHopperPosition(0.2);
        sleep(500);
//        while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
//            subsystems.shooter.setHopperPosition(0.0);
//            subsystems.shooter.setPowers(0.895);
//            subsystems.drivetrain.aim(2, 0.02, 0.015);
//        }
//        subsystems.drivetrain.run(0, 0);
//        sleep(100);
//        subsystems.shooter.setHopperPosition(0.2);
//        sleep(500);
//        subsystems.drivetrain.resetAiming();
//        while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
//            subsystems.shooter.setHopperPosition(0.0);
//            subsystems.shooter.setPowers(0.895);
//            subsystems.drivetrain.aim(3, 0.01, 0.015);
//        }
//        subsystems.drivetrain.run(0, 0);
//        sleep(100);
//        subsystems.shooter.setHopperPosition(0.2);
//        sleep(500);
//        subsystems.drivetrain.resetAiming();
        subsystems.shooter.setHopperPosition(0.0);
        subsystems.shooter.setPowers(0.0);
        subsystems.shooter.setPowerShotPosition(0.65);
        controller.startNextSection();

        if(path == 0) {
            //between while loops is where we call startNextSection() and everything works out great omg
            while (controller.getCurrentSection() == 2 && !isStopRequested()) {
                subsystems.wobbleGrabber.wobbleTurn.setPosition(0.05);
            }
            subsystems.wobbleGrabber.wobbleTurn.setPosition(0.05);
            subsystems.wobbleGrabber.wobbleGrab.setPosition(1.5);
            sleep(500);
            controller.startNextSection();
            while (controller.getCurrentSection() == 3 && !isStopRequested()) {

            }
            subsystems.wobbleGrabber.wobbleGrab.setPosition(0.05);
            sleep(500);
            controller.startNextSection();
            while (controller.getCurrentSection() == 4 && !isStopRequested()) {
            }
            subsystems.wobbleGrabber.wobbleTurn.setPosition(0.05);
            subsystems.wobbleGrabber.wobbleGrab.setPosition(1.5);
            sleep(500);
            controller.startNextSection();
            while (controller.getCurrentSection() == 5 && !isStopRequested()) {
                subsystems.wobbleGrabber.wobbleTurn.setPosition(0.52);
            }
        }
        else if(path == 1) {
           while(controller.getCurrentSection() == 2 && !isStopRequested()) {
               subsystems.wobbleGrabber.wobbleTurn.setPosition(0.05);
           }
           subsystems.wobbleGrabber.wobbleTurn.setPosition(0.05);
           subsystems.wobbleGrabber.wobbleGrab.setPosition(1.5);
           sleep(500);
           controller.startNextSection();
           while(controller.getCurrentSection() == 3 && !isStopRequested()) {
               subsystems.intake.run(true, false, false);
           }
           controller.startNextSection();
           while(controller.getCurrentSection() == 4 && !isStopRequested()) {

           }
            subsystems.intake.run(false, false, false);
           subsystems.wobbleGrabber.wobbleGrab.setPosition(0.05);
           sleep(500);
           controller.startNextSection();
           while(controller.getCurrentSection() == 5 && !isStopRequested()) {
               subsystems.shooter.setPowers(0.895);
               subsystems.shooter.setHopperPosition(0.0);
               subsystems.wobbleGrabber.wobbleTurn.setPosition(0.52);
           }
           subsystems.drivetrain.resetAiming();
           while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
               subsystems.drivetrain.aim(0, 0.1, 0.009);
           }
           subsystems.drivetrain.run(0, 0);
           sleep(100);
           subsystems.shooter.setHopperPosition(0.16);
           sleep(500);
           controller.startNextSection();
           while(controller.getCurrentSection() == 6 && !isStopRequested()) {
                subsystems.wobbleGrabber.wobbleTurn.setPosition(0.05);
           }
           subsystems.wobbleGrabber.wobbleTurn.setPosition(0.05);
           subsystems.wobbleGrabber.wobbleGrab.setPosition(1.5);
           sleep(500);
           controller.startNextSection();
           while(controller.getCurrentSection() == 7 && !isStopRequested()) {
               subsystems.wobbleGrabber.wobbleGrab.setPosition(0.05);
               subsystems.wobbleGrabber.wobbleTurn.setPosition(0.52);
           }

        }
        else if(path == 2) {
            while(controller.getCurrentSection() == 2 && !isStopRequested()) {
                subsystems.wobbleGrabber.wobbleTurn.setPosition(0.52);
            }
            subsystems.wobbleGrabber.wobbleTurn.setPosition(0.05);
            subsystems.wobbleGrabber.wobbleGrab.setPosition(1.5);
            sleep(500);
            controller.startNextSection();
            while(controller.getCurrentSection() == 3 && !isStopRequested()) {}
            sleep(100);
            controller.startNextSection();
            while(controller.getCurrentSection() == 4 && !isStopRequested()) {
//                subsystems.intake.run(false, true, false);
//                subsystems.intake.setIntakePower(-1.0);
            }
            subsystems.intake.run(true, false, false);
            sleep(200);
            controller.startNextSection();
            while(controller.getCurrentSection() == 5 && !isStopRequested()) {
//                subsystems.intake.run(true, false, false);
            }
            subsystems.wobbleGrabber.wobbleGrab.setPosition(0.05);
            sleep(500);
            controller.startNextSection();
            while(controller.getCurrentSection() == 6 && !isStopRequested()) {
                subsystems.shooter.setPowers(0.8);
                subsystems.shooter.setHopperPosition(0.0);
                subsystems.wobbleGrabber.wobbleTurn.setPosition(0.52);
            }
            subsystems.intake.run(false, false, false);
            subsystems.drivetrain.resetAiming();
            while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
                subsystems.intake.run(false, false, true);
                subsystems.drivetrain.aim(0, 0.1, 0.009);
            }
            subsystems.drivetrain.run(0, 0);
            sleep(100);
            subsystems.shooter.setHopperPosition(0.2);
            sleep(200);
            subsystems.shooter.setHopperPosition(0.0);
            sleep(200);
            subsystems.shooter.setHopperPosition(0.2);
            sleep(200);
            subsystems.shooter.setHopperPosition(0.0);
            sleep(200);
            subsystems.shooter.setHopperPosition(0.2);
            sleep(500);
            controller.startNextSection();
            while(controller.getCurrentSection() == 7 && !isStopRequested()) {
                subsystems.wobbleGrabber.wobbleTurn.setPosition(0.05);
            }
            subsystems.wobbleGrabber.wobbleTurn.setPosition(0.05);
            subsystems.wobbleGrabber.wobbleGrab.setPosition(1.5);
            sleep(500);
            controller.startNextSection();
            while(controller.getCurrentSection() == 8 && !isStopRequested()) {
                subsystems.wobbleGrabber.wobbleGrab.setPosition(0.05);
                subsystems.wobbleGrabber.wobbleTurn.setPosition(0.52);
            }
        }

        Robot.storeRobotPosition();

    }
}
