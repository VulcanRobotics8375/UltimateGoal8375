package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.robot.Robot;

@TeleOp(name = "main", group = "main")
public class Main extends TeleOpPipeline {
    @Override
    public void init() {
        ip = "";
        dash = false;
        teleopInit();
    }

    @Override
    public void loop() {

        subsystems.intake.run(gamepad2.a, gamepad2.right_bumper);

        subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x);

        subsystems.wobbleGrabber.run(-gamepad2.right_stick_y, gamepad2.dpad_right, gamepad2.dpad_left);

        subsystems.shooter.run(gamepad2.x, gamepad2.left_stick_y);

        telemetry.update();

    }
}
