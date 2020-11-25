package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        telemetry.addLine("ready");
        telemetry.update();

        setStart(new Point(138.34, 21.6), 0);

        waitForStart();
        telemetry.addLine("starting");
        telemetry.update();

        Robot.startOdometryThread();

        Mecanum motionProfile = (Mecanum) Robot.motionProfile;

        while(opModeIsActive()) {
//            subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_bumper, gamepad1.a, gamepad1.x, gamepad1.y, gamepad1.b);
            subsystems.drivetrain.tunePID(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.right_bumper);
            telemetry.update();
        }

    }
}
