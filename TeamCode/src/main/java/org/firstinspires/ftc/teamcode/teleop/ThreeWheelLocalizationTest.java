package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;

@TeleOp(name = "roadrunner localization test", group = "test")
public class ThreeWheelLocalizationTest extends TeleOpPipeline {

    public void runOpMode() throws InterruptedException {
        StandardTrackingWheelLocalizer drive = new StandardTrackingWheelLocalizer(hardwareMap);
        dash = false;
        teleopInit();


        waitForStart();

        while(opModeIsActive()) {
            drive.update();

            subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, false, false, gamepad1.y, gamepad1.x, gamepad1.left_bumper, gamepad1.right_bumper);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("theta", poseEstimate.getHeading());
            telemetry.update();

        }

    }

}
