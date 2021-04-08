package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.vulcanrobotics.robotcorelib.drive.StandardTrackingWheelLocalizer;
import org.vulcanrobotics.robotcorelib.framework.AutoPipeline;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.PathPoint;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.PurePursuit;
import org.vulcanrobotics.robotcorelib.robot.Robot;
import org.vulcanrobotics.robotcorelib.vision.StackDetectorCV;

import java.util.ArrayList;

@Autonomous(name = "NorCal Auto", group = "main", preselectTeleOp = "main")
public class NorCalAuto extends AutoPipeline {

    StandardTrackingWheelLocalizer drive;
    private int autoPath = 2;

    @Override
    public void runOpMode() {
        try {
            autoInit();
        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }
        StackDetectorCV stackDetector = new StackDetectorCV();
//        initVision(stackDetector);
        subsystems.wobbleGrabber.wobbleGrab.setPosition(0.05);
        drive = new StandardTrackingWheelLocalizer(hardwareMap);
        drive.setPoseEstimate(new Pose2d(21.6, 108, 0.0));

        ArrayList<ArrayList<PathPoint>> sections = new ArrayList<>();
        //path points here
        ArrayList<PathPoint> start1 = new ArrayList<>();
        start1.add(new PathPoint(108, 21.6, -0.5, -1, 12, 0));
        start1.add(new PathPoint(108, 21.6, -0.5, -1, 12, 0));
        start1.add(new PathPoint(150, 80, -0.5, -1, 12, 0));
//        start1.add(new PathPoint(100, 170, -0.5, 1, 12, 0));

        ArrayList<PathPoint> start2 = new ArrayList<>();
        start2.add(new PathPoint(149, 170, -0.5, -1, 12, 0));

        /*
         * CASE ZERO
         */
        ArrayList<PathPoint> section1 = new ArrayList<>();
        section1.add(new PathPoint(149, 170, -1, -1, 15, 0));
        section1.add(new PathPoint(50, 180, -1, -1, 15, 0));

        ArrayList<PathPoint> section2 = new ArrayList<>();
//        section2.add(new PathPoint(185, 180, -1, 1, 15, 0));
        section2.add(new PathPoint(55, 165, -1, -1, 10, 0.1));
        section2.add(new PathPoint(55, 165, -1, -1, 10, 0.1));
        section2.add(new PathPoint(35, 150, -0.5, -1, 10, Math.PI - 0.05));

        ArrayList<PathPoint> section3 = new ArrayList<>();
        section3.add(new PathPoint(35, 150, -0.5, -1, 15, 2.0 * Math.PI));
        section3.add(new PathPoint(50, 165, -0.7, -1, 15, 2.0 * Math.PI));

        ArrayList<PathPoint> section4 = new ArrayList<>();
        section4.add(new PathPoint(50, 165, -0.7, -1, 12, 2.0 * Math.PI));
        section4.add(new PathPoint(100, 185, -0.7, -1, 12, 2.0 * Math.PI));
        section4.add(new PathPoint(145, 210, -1, -1, 12, 2.0 * Math.PI));

        /*
         * CASE ONE
         */
        ArrayList<PathPoint> sectionB1 = new ArrayList<>();
        sectionB1.add(new PathPoint(149, 170, -0.5, -1, 12, 0));
        sectionB1.add(new PathPoint(149, 170, -0.5, -1, 12, 0));
        sectionB1.add(new PathPoint(103, 230, -0.5, -1, 12, 0));

        ArrayList<PathPoint> sectionB2 = new ArrayList<>();
        sectionB2.add(new PathPoint(150, 170, -1, -1, 12, 0));
        sectionB2.add(new PathPoint(160, 93, -1, -1, 12, 0));

        ArrayList<PathPoint> sectionB3 = new ArrayList<>();
        sectionB3.add(new PathPoint(132, 98, -0.25, -1, 12, (3.0 * Math.PI) / 2.0));

        ArrayList<PathPoint> sectionB4 = new ArrayList<>();
        sectionB4.add(new PathPoint(95, 155, -0.5, -1, 12, 0));

        ArrayList<PathPoint> sectionB5 = new ArrayList<>();
        sectionB5.add(new PathPoint(103, 215, -0.5, -1, 12, 0));

        ArrayList<PathPoint> sectionB6 = new ArrayList<>();
        sectionB6.add(new PathPoint(103, 220, -1, -1, 12, 0));
        sectionB6.add(new PathPoint(103, 220, -1, -1, 12, 0));
        sectionB6.add(new PathPoint(145, 210, -0.3, -1, 12, 0));

        /*
         * CASE TWO
         */
        ArrayList<PathPoint> sectionC1 = new ArrayList<>();
//        sectionC1.add(new PathPoint(149, 170, -1, -1, 12, 0));
//        sectionC1.add(new PathPoint(149, 170, -1, -1, 12, 0));
        sectionC1.add(new PathPoint(65, 290, -1, -1, 12, 0));

        ArrayList<PathPoint> sectionCW1 = new ArrayList<>();
        sectionCW1.add(new PathPoint(140, 170, -0.8, -1, 12, 0));
        sectionCW1.add(new PathPoint(150, 91, -0.8, -1, 12, 0));

        ArrayList<PathPoint> sectionCW2 = new ArrayList<>();
        sectionCW2.add(new PathPoint(135, 96, -0.25, -1, 12, (3.0 * Math.PI) / 2.0));

        ArrayList<PathPoint> sectionC2 = new ArrayList<>();
        sectionC2.add(new PathPoint(102, 93, -0.2, -1, 12, 0));

        ArrayList<PathPoint> sectionC3 = new ArrayList<>();
        sectionC3.add(new PathPoint(102, 155, -0.5, -1, 12, 0));

        ArrayList<PathPoint> sectionC4 = new ArrayList<>();
        sectionC4.add(new PathPoint(60, 280, -1, -1, 12, 0));

        ArrayList<PathPoint> sectionC5 = new ArrayList<>();
//        sectionC5.add(new PathPoint(70, 290, -1, -1, 12, 0));
        sectionC5.add(new PathPoint(60, 280, -1, -1, 12, 0));
        sectionC5.add(new PathPoint(110, 200, -1, -1, 12, 0));
        sectionC5.add(new PathPoint(120, 210, -0.2, -1, 12, 0));

        waitForStart();

        if(autoPath == 0) {
            sections.add(start1);
            sections.add(start2);
            sections.add(section1);
            sections.add(section2);
            sections.add(section3);
            sections.add(section4);
        }
        else if(autoPath == 1) {
            sections.add(start1);
            sections.add(start2);
            sections.add(sectionB1);
            sections.add(sectionB2);
            sections.add(sectionB3);
            sections.add(sectionB4);
            sections.add(sectionB5);
            sections.add(sectionB6);

        }
        else if(autoPath == 2) {
            sections.add(start1);
            sections.add(start2);
            sections.add(sectionC1);
            sections.add(sectionCW1);
            sections.add(sectionCW2);
            sections.add(sectionC2);
            sections.add(sectionC3);
            sections.add(sectionC4);
            sections.add(sectionC5);
        }

        final PurePursuit controller = new PurePursuit(sections);
        super.controller = controller;

        startInterruptHandler();

        //path follower thread
        new Thread(new Runnable() {
            @Override
            public void run() {
                //potential sleep here
                if(!isStopRequested()) {
                    controller.run();
                }
            }
        }).start();

        //odo thread
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()) {
                    updateOdometry();
//                    telemetry.addData("x", Robot.getRobotX());
//                    telemetry.addData("y", Robot.getRobotY());
//                    telemetry.addData("theta", Robot.getRobotAngleRad());
//                    telemetry.update();
                }
            }
        }).start();

        while(controller.getCurrentSection() == 0) {
            //subsystem code goes here
            subsystems.shooter.setPowers(0.675);

        }

        controller.startNextSection();
        while(controller.getCurrentSection() == 1) {
            subsystems.shooter.setPowers(0.675);
        }

        while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
            subsystems.shooter.setHopperPosition(0.0);
            subsystems.shooter.setPowers(0.675);
            subsystems.drivetrain.aim(1, 0.0, 0.01);
        }
        subsystems.drivetrain.run(0, 0);
        sleep(100);
        subsystems.shooter.setHopperPosition(0.2);
        sleep(500);
        subsystems.drivetrain.resetAiming();
        while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
            subsystems.shooter.setHopperPosition(0.0);
            subsystems.shooter.setPowers(0.675);
            subsystems.drivetrain.aim(2, 0.0, 0.01);
        }
        subsystems.drivetrain.run(0, 0);
        sleep(100);
        subsystems.shooter.setHopperPosition(0.2);
        sleep(500);
        subsystems.drivetrain.resetAiming();
        while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
            subsystems.shooter.setHopperPosition(0.0);
            subsystems.shooter.setPowers(0.675);
            subsystems.drivetrain.aim(3, 0.0, 0.01);
        }
        subsystems.drivetrain.run(0, 0);
        sleep(100);
        subsystems.shooter.setHopperPosition(0.2);
        sleep(500);
        subsystems.drivetrain.resetAiming();
        subsystems.shooter.setPowers(0.0);
        controller.startNextSection();

        switch (autoPath) {
            case 0:
                while (controller.getCurrentSection() == 2) {
                    subsystems.wobbleGrabber.setTurnPosition(0.05);
                }
                subsystems.wobbleGrabber.setGrabPosition(1.5);
                sleep(500);
//        subsystems.wobbleGrabber.setTurnPosition(0.65);
                controller.startNextSection();
                while (controller.getCurrentSection() == 3) {

                }
                subsystems.wobbleGrabber.setGrabPosition(0.05);
                sleep(500);
                controller.startNextSection();
                while (controller.getCurrentSection() == 4) {
                }
                subsystems.wobbleGrabber.setGrabPosition(1.5);
                sleep(500);
                controller.startNextSection();
                while (controller.getCurrentSection() == 5) {
                    subsystems.wobbleGrabber.setTurnPosition(0.65);
                }

                //store robot position to file after auto finishes
                return;
            case 1:
                while(controller.getCurrentSection() == 2) {
                    subsystems.wobbleGrabber.setTurnPosition(0.05);
                }
                subsystems.wobbleGrabber.setGrabPosition(1.5);
                sleep(500);
                controller.startNextSection();
                while(controller.getCurrentSection() == 3) {

                }
                sleep(500);
                controller.startNextSection();
                while(controller.getCurrentSection() == 4) {

                }
                subsystems.wobbleGrabber.setGrabPosition(0.05);
                sleep(500);
                controller.startNextSection();
                while(controller.getCurrentSection() == 5) {
//                    subsystems.wobbleGrabber.setTurnPosition(0.65);
                    subsystems.wobbleGrabber.setGrabPosition(0.05);
                    subsystems.shooter.setPowers(0.805);
                    subsystems.intake.setIntakePower(-1.0);
                    subsystems.intake.setTransferPower(1.0);
                    subsystems.shooter.setHopperPosition(0.0);
                }
                while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
                    subsystems.drivetrain.aim(0, 0.0, 0.015);
                    telemetry.addLine("aiming");
                    telemetry.update();
                }
                subsystems.shooter.setHopperPosition(0.2);
                sleep(500);
                subsystems.drivetrain.resetAiming();
                controller.startNextSection();
                while(controller.getCurrentSection() == 6) {
                    subsystems.wobbleGrabber.setTurnPosition(0.01);
                }
                subsystems.wobbleGrabber.setGrabPosition(1.5);
                sleep(1000);
                subsystems.wobbleGrabber.setTurnPosition(0.65);
                controller.startNextSection();
                while(controller.getCurrentSection() == 7) {

                }
//                sleep(500);
//                controller.startNextSection();
                return;

            case 2:
                while(controller.getCurrentSection() == 2) {
                    subsystems.wobbleGrabber.setTurnPosition(0.01);
                }
                subsystems.wobbleGrabber.setGrabPosition(1.5);
                sleep(500);
                controller.startNextSection();
                while(controller.getCurrentSection() == 3) {

                }
                controller.startNextSection();
                while(controller.getCurrentSection() == 4) {

                }
                subsystems.wobbleGrabber.setGrabPosition(0.05);
                sleep(500);
                subsystems.wobbleGrabber.setTurnPosition(0.65);
                sleep(500);
                controller.startNextSection();
                while(controller.getCurrentSection() == 5) {
                    subsystems.wobbleGrabber.setGrabPosition(0.05);
                    subsystems.shooter.setPowers(0.78);
                    subsystems.intake.setIntakePower(1.0);
                    subsystems.intake.setTransferPower(1.0);
                    subsystems.shooter.setHopperPosition(0.0);
                }
                subsystems.intake.setDeployPosition(0.5);
                sleep(2000);
                while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
                    subsystems.drivetrain.aim(0, 0.0, 0.015);
                }
                subsystems.drivetrain.run(0, 0);
                subsystems.shooter.setHopperPosition(0.2);
                sleep(100);
                subsystems.shooter.setHopperPosition(0.0);
                sleep(150);
                subsystems.shooter.setHopperPosition(0.2);
                sleep(100);
                subsystems.shooter.setHopperPosition(0.0);
                subsystems.intake.setIntakePower(-1.0);
                subsystems.drivetrain.resetAiming();
                controller.startNextSection();
                while(controller.getCurrentSection() == 6) {
                    subsystems.shooter.setPowers(0.805);
//                    telemetry.addLine("section 6");
//                    telemetry.update();
                }
                while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
                    subsystems.drivetrain.aim(0, 0.0, 0.015);
//                    telemetry.addLine("aiming");
//                    telemetry.update();
                }
//                telemetry.update();
                subsystems.drivetrain.run(0, 0);
                subsystems.shooter.setHopperPosition(0.2);
                sleep(100);
                subsystems.shooter.setHopperPosition(0.0);
                sleep(100);
                subsystems.shooter.setHopperPosition(0.2);
                sleep(100);
                subsystems.shooter.setHopperPosition(0.0);
                sleep(100);
                subsystems.shooter.setHopperPosition(0.2);
                sleep(100);
                subsystems.shooter.setHopperPosition(0.0);
                controller.startNextSection();

                while(controller.getCurrentSection() == 7) {
                    subsystems.intake.setIntakePower(0);
                    subsystems.intake.setTransferPower(0);
                    subsystems.wobbleGrabber.setTurnPosition(0.01);
                }
                subsystems.wobbleGrabber.setGrabPosition(1.5);
                sleep(500);
                controller.startNextSection();

                while(controller.getCurrentSection() == 8) {
                    subsystems.wobbleGrabber.setTurnPosition(0.65);
                }
                sleep(500);
                return;
        }

        Robot.storeRobotPosition();

    }

    private void updateOdometry() {
        //rr update
        drive.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
        Pose2d poseVelocity = drive.getPoseVelocity();

        //system update
        Robot.setRobotPos(new Point(poseEstimate.getY(), poseEstimate.getX()));
        Robot.setRobotAngle(poseEstimate.getHeading());
        Robot.setRobotVelocity(new Point(poseVelocity.getY(), poseVelocity.getX()));

    }

}
