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



        telemetry.update();

    }
}
