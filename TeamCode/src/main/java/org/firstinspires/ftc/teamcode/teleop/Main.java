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

        subsystems.intake.run(gamepad2.a, gamepad2.b);

        //TODO update this when shooter code is done
        subsystems.shooter.run();

    }
}
