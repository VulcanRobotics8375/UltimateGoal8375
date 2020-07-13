package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.vulcanrobotics.robotcorelib.Dashboard.Dashboard;
import org.vulcanrobotics.robotcorelib.Dashboard.Hardware.DashboardMotor;

@TeleOp(name = "dashTest", group = "yeet")
public class ExOpMode extends OpMode {
    String[] data;

    @Override
    public void init() {
        Dashboard.connect("10.0.0.43", 8375);
        while(!Dashboard.running) {}
        DashboardMotor motor = new DashboardMotor(0);

    }

    public void start() {
        Dashboard.start();
    }

    @Override
    public void loop() {
        
//        Dashboard.sendToDash(data);
    }

    public void stop() {
        Dashboard.running = false;
    }
}
