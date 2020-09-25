package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.robot.Robot;

@TeleOp(name = "main", group = "main")
public class Main extends TeleOpPipeline {

    public void runOpMode() {

        dash = false;
        ip = "";
        teleopInit();

        waitForStart();

        while (opModeIsActive()) {
            boolean shoot = gamepad1.a;

            subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper, shoot);


            telemetry.update();
        }

    }

}
