package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.vulcanrobotics.robotcorelib.drive.StandardTrackingWheelLocalizer;
import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;

public class OdometryFineTuner extends TeleOpPipeline {
    boolean running;

    @Override
    public void runOpMode() {

        teleopInit();
        running = true;

        waitForStart();

        telemetry.addLine("Odometry Fine Tuner - Vulcan Robotics");
        telemetry.addLine("Press A to start");
        telemetry.update();
        while(!gamepad1.a) {}
        while(gamepad1.a) {}

        while(running) {
            StandardTrackingWheelLocalizer drive = new StandardTrackingWheelLocalizer(hardwareMap);
            Pose2d poseEstimate = drive.getPoseEstimate();
            while (!gamepad1.a) {
                subsystems.drivetrain.run(gamepad1.left_stick_y, gamepad1.right_stick_x);
                drive.update();
                poseEstimate = drive.getPoseEstimate();
                telemetry.addLine("spin the robot around EXACTLY 10 times, then press A");
                telemetry.addData("robot angle", poseEstimate.getHeading());
                telemetry.addData("wheel base", StandardTrackingWheelLocalizer.LATERAL_DISTANCE);
                telemetry.update();
            }
            subsystems.drivetrain.run(0, 0);
            while (gamepad1.a) {}

            double totalError = poseEstimate.getHeading() < Math.PI ? poseEstimate.getHeading() : poseEstimate.getHeading() - (2.0 * Math.PI);
            double percentError = ((totalError) - (10.0 * 2.0 * Math.PI)) / (10.0 * 2.0 * Math.PI);
            double currentWheelBase = StandardTrackingWheelLocalizer.LATERAL_DISTANCE;
            double newWheelBase = currentWheelBase * (1 + percentError);
            StandardTrackingWheelLocalizer.LATERAL_DISTANCE = newWheelBase;

            telemetry.addData("total error", totalError);
            telemetry.addData("percent error", percentError * 100.0);
            telemetry.addData("new wheelbase", newWheelBase);

            telemetry.addLine();
            telemetry.addLine("press A to continue, or B to exit");
            telemetry.update();
            while (!gamepad1.a && !gamepad1.b) {
                if (gamepad1.b) {
                    running = false;
                }
            }
            while (gamepad1.a) {
            }
        }


    }
}
