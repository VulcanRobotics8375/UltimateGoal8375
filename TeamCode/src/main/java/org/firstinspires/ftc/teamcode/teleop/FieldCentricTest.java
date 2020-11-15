package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        Robot.startOdometryThread();

        while(opModeIsActive()) {
            subsystems.drivetrain.fieldCentricMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("robot x", Robot.getRobotX());
            telemetry.addData("robot y", Robot.getRobotY());


            telemetry.update();
        }

    }
}
