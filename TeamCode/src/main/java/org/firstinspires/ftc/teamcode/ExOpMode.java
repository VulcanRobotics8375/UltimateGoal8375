package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.vulcanrobotics.robotcorelib.dashboard.Dashboard;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.DashboardMotor;
import org.vulcanrobotics.robotcorelib.robot.Robot;

@TeleOp(name = "dashTest", group = "yeet")
public class ExOpMode extends OpMode {


    @Override
    public void init() {
//        Dashboard.connect("10.0.0.43", 8375);
//        while(!Dashboard.running) {}
    }

    public void start() {
        Dashboard.start();
    }

    @Override
    public void loop() {

        double power = gamepad1.a ? 1 : 0;

       double adjustPower = gamepad1.right_trigger - gamepad1.left_trigger;



//        Dashboard.sendToDash(data);
    }

    public void stop() {
        Dashboard.running = false;
    }
}
