package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;

@TeleOp(name = "main", group = "main")
public class Main extends TeleOpPipeline {

    public void runOpMode() {

        dash = false;
        ip = "";
        teleopInit();

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();
        telemetry.addLine("starting");
        telemetry.update();

        while (opModeIsActive()) {
            boolean shoot = gamepad1.a;
            boolean powerShot = gamepad1.b;

            subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.left_bumper, shoot, powerShot);
            subsystems.intake.run(gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y);


            telemetry.update();
        }

    }

}
