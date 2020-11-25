package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.robot.Robot;

@TeleOp(name = "wtf", group = "wtf")
public class TestCrashes extends TeleOpPipeline {


    @Override
    public void runOpMode() throws InterruptedException {

        dash = false;
//        teleopInit();
       Robot.test();

        waitForStart();

        while(opModeIsActive()) {

        }
    }
}
