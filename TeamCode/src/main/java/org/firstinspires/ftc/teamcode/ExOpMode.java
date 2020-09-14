package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.robot.Robot;

@TeleOp(name = "dashTest", group = "yeet")
public class ExOpMode extends TeleOpPipeline {

    @Override
    public void init() {
        //ip for dashboard and dashboard toggle
        ip = "10.0.0.43";
        dash = false;
        teleopInit();

        //init code here

    }

    public void start() {
       teleopStart();

       //start code here

    }

    @Override
    public void loop() {
        teleopLoop();

        /*
        Robot code goes here
        To access subsystems, use Robot.getComponents().subsystem
        Example:
         */
        Robot.getComponents().drivetrain.move(0, 0);


    }

    public void stop() {
        teleopStop();
    }
}
