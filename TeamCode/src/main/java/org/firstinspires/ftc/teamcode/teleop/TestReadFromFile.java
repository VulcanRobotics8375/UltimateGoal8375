package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.robot.Robot;

@TeleOp(name = "read test", group = "test")
public class TestReadFromFile extends TeleOpPipeline {
    @Override
    public void runOpMode() throws InterruptedException {

        teleopInit();
        Robot.loadRobotPosition();

        waitForStart();
        Robot.startOdometryThread();

        while(opModeIsActive()) {
            telemetry.addData("x", Robot.getRobotX());
            telemetry.addData("y", Robot.getRobotY());
            telemetry.addData("theta", Robot.getRobotAngleDeg());
            telemetry.update();
        }
        Robot.stopOdometryThread();

    }
}
