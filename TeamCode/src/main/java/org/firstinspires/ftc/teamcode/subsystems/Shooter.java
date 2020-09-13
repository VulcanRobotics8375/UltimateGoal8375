package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.vulcanrobotics.robotcorelib.dashboard.hardware.DashboardMotor;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.DashboardServo;
import org.vulcanrobotics.robotcorelib.subsystem.Subsystem;


public class Shooter extends Subsystem {


    @Override
    public void init() {

    }

    public void run() {

    }

    @Override
    public void stop() {

    }


    public void setServoAngle(Servo servo, double angle, double maxAngle) {
        servo.setPosition(angle / maxAngle);
    }
}
