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
        initVision(stackDetector);
        subsystems.wobbleGrabber.wobbleGrab.setPosition(0.05);
        drive = new StandardTrackingWheelLocalizer(hardwareMap);
        drive.setPoseEstimate(new Pose2d(21.6, 108, 0.0));

        ArrayList<ArrayList<PathPoint>> sections = new ArrayList<>();
        //path points here

        waitForStart();

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
                }
            }
        }).start();

        while(controller.getCurrentSection() == 0) {
            //subsystem code goes here
            
        }


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
