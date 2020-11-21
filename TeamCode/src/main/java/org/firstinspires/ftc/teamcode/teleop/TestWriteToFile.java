package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.robot.Robot;

@TeleOp(name = "write to file test", group = "test")
public class TestWriteToFile extends TeleOpPipeline {
    @Override
    public void runOpMode() throws InterruptedException {

        dash = false;
        ip = "";
        teleopInit();

        waitForStart();
        Robot.startOdometryThread();

        while(opModeIsActive()) {

        }

        Robot.stopOdometryThread();
        Robot.storeRobotPosition();


    }
}
