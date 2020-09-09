package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.vulcanrobotics.robotcorelib.dashboard.hardware.DashboardMotor;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.DashboardServo;
import org.vulcanrobotics.robotcorelib.subsystem.Subsystem;


public class Shooter extends Subsystem {

    public DashboardMotor shooter;
    public DashboardServo adjust;
    ElapsedTime adjustTimer = new ElapsedTime();
    double lastAdjustPosition;
    double shooterHighLimit = 800;
    double limitRange = 200;

    @Override
    public void init() {
        shooter = new DashboardMotor(hardwareMap.dcMotor.get("shooter"));
        shooter.getMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        adjust = new DashboardServo(hardwareMap.servo.get("adjust"));
        adjustTimer.reset();

    }

    public void run(double power, double adjustPower) {


       if(adjustTimer.milliseconds() > 100) {
          adjust.getServo().setPosition(lastAdjustPosition + (0.01 * Math.signum(adjustPower)));
          lastAdjustPosition = adjust.getServo().getPosition();
          adjustTimer.reset();
       }

        shooter.setPower(power);
    }

    @Override
    public void stop() {

    }


    public void setServoAngle(Servo servo, double angle, double maxAngle) {
        servo.setPosition(angle / maxAngle);
    }
}
