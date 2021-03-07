package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.vulcanrobotics.robotcorelib.drive.StandardTrackingWheelLocalizer;
import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.robot.Robot;

@TeleOp(name = "field centric test", group = "test")
public class FieldCentricTest extends TeleOpPipeline {


    @Override
    public void runOpMode() {
        dash = false;
        ip = "";
        teleopInit();

        telemetry.addLine("ready");
        telemetry.update();

        setStart(new Point(0, 0), 0);

        waitForStart();
        telemetry.addLine("starting");
        telemetry.update();

//        Robot.startOdometryThread();
        StandardTrackingWheelLocalizer drive = new StandardTrackingWheelLocalizer(hardwareMap);


        while(opModeIsActive()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            Robot.setRobotPos(new Point(poseEstimate.getY(), poseEstimate.getX()));
            Robot.setRobotAngle(poseEstimate.getHeading());
            subsystems.drivetrain.fieldCentricMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("robot x", Robot.getRobotX());
            telemetry.addData("robot y", Robot.getRobotY());


            telemetry.update();
        }

    }
}
