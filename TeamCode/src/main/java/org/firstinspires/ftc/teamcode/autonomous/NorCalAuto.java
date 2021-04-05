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
        start1.add(new PathPoint(137, 21.6, -1, -1, 12, 0));
        start1.add(new PathPoint(137, 21.6, -1, -1, 12, 0));
        start1.add(new PathPoint(95, 80, -1, -1, 12, 0));
//        start1.add(new PathPoint(100, 170, -0.5, 1, 12, 0));

        ArrayList<PathPoint> start2 = new ArrayList<>();
        start2.add(new PathPoint(100, 170, -0.5, -1, 12, 0));

        ArrayList<PathPoint> section1 = new ArrayList<>();
        section1.add(new PathPoint(145, 170, -1, -1, 12, 0));
        section1.add(new PathPoint(60, 180, -1, -1, 15, 0));

        ArrayList<PathPoint> section2 = new ArrayList<>();
//        section2.add(new PathPoint(185, 180, -1, 1, 15, 0));
        section2.add(new PathPoint(60, 150, -1, -1, 15, 0));
        section2.add(new PathPoint(60, 150, -1, -1, 15, 0));
        section2.add(new PathPoint(35, 100, -0.5, -1, 15, Math.PI));

        ArrayList<PathPoint> section3 = new ArrayList<>();
        section3.add(new PathPoint(70, 155, -0.7, -1, 12, 2.0 * Math.PI));

        ArrayList<PathPoint> section4 = new ArrayList<>();
        section4.add(new PathPoint(145, 195, -1, -1, 12, 2.0 * Math.PI));

        waitForStart();

        sections.add(start1);
        sections.add(start2);

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
            subsystems.drivetrain.aim(1, 0.0, 0.015);
        }
        subsystems.drivetrain.run(0, 0);
        sleep(100);
        subsystems.shooter.setHopperPosition(0.2);
        sleep(500);
        subsystems.drivetrain.resetAiming();
        while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
            subsystems.shooter.setHopperPosition(0.0);
            subsystems.shooter.setPowers(0.675);
            subsystems.drivetrain.aim(2, 0.0, 0.015);
        }
        subsystems.drivetrain.run(0, 0);
        sleep(100);
        subsystems.shooter.setHopperPosition(0.2);
        sleep(500);
        subsystems.drivetrain.resetAiming();
        while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
            subsystems.shooter.setHopperPosition(0.0);
            subsystems.shooter.setPowers(0.675);
            subsystems.drivetrain.aim(3, 0.0, 0.015);
        }
        subsystems.drivetrain.run(0, 0);
        sleep(100);
        subsystems.shooter.setHopperPosition(0.2);
        sleep(500);
        subsystems.drivetrain.resetAiming();

        //store robot position to file after auto finishes
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
