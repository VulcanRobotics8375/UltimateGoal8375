package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.vulcanrobotics.robotcorelib.drive.StandardTrackingWheelLocalizer;
import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.Mecanum;
import org.vulcanrobotics.robotcorelib.robot.Robot;

@TeleOp(name = "aimbot pid tuner", group = "test")
public class DrivetrainPidTuner extends TeleOpPipeline {
    @Override
    public void runOpMode() {
        dash = false;
        ip = "";
        teleopInit();

        ElapsedTime timer = new ElapsedTime();
        double time;

        telemetry.addLine("ready");
        telemetry.update();

//        setStart(new Point(138.34, 21.6), 0);
        StandardTrackingWheelLocalizer drive = new StandardTrackingWheelLocalizer(hardwareMap);
        drive.setPoseEstimate(new Pose2d(21.6, 108, 0.0));

        waitForStart();
        telemetry.addLine("starting");
        telemetry.update();

//        Robot.startOdometryThread();

//        Mecanum motionProfile = (Mecanum) Robot.motionProfile;

        while(opModeIsActive()) {
            timer.reset();
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            Robot.setRobotPos(new Point(poseEstimate.getY(), poseEstimate.getX()));
            Robot.setRobotAngle(poseEstimate.getHeading());
            subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.a, gamepad1.b, gamepad1.y, gamepad1.x, gamepad1.left_bumper, gamepad1.right_bumper);
            subsystems.drivetrain.tunePID(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.right_bumper);
            telemetry.update();

            double TARGET_UPS = 50.0;
            while(timer.milliseconds() < 1000.0 / TARGET_UPS) {
//               sleep(1);
            }
//            time = timer.milliseconds();

        }

    }
}
