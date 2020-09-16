package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Shooter {

    public DcMotor shooter;
    private double shooterPower = 1.0;

    public Shooter(DcMotor shooter) {
        this.shooter = shooter;
    }

    public void init() {
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection((DcMotor.Direction.FORWARD));
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void run(boolean on) {
        if(on)
            shooter.setPower(shooterPower);
        else
            shooter.setPower(0);
    }
}
