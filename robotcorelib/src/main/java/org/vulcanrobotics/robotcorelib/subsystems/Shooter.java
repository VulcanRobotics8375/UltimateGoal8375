package org.vulcanrobotics.robotcorelib.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public class Shooter extends Subsystem {
    public DcMotor shooter;
    public Servo hopper;
    private boolean hopperButton;
    private int hopperOn = 1;
    private double hopperBeforeTime;
    private boolean hopperOut;
    private boolean shooterButton;
    //lowest shooter power when closest is 0.8, highest shooter power when furthest is
    public double shooterPower;
    //on = goal, off = powershot
    private boolean shooterMode;
    private double shooterModeNum = 88.9;
    private double shooterPowerLeft;
    private double shooterPowerRight;
    private int shooterOn = -1;

    public Shooter() {
    }
    @Override
    public void init() {
        shooter = hardwareMap.dcMotor.get("shooter");
        hopper = hardwareMap.servo.get("hopper");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection((DcMotor.Direction.REVERSE));
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void run(boolean shooterButton, boolean hopperButton, boolean shooterMode) {
        if (shooterButton && !this.shooterButton) {
            shooterOn *= -1;
            this.shooterButton = true;
        }
        if (!shooterButton && this.shooterButton) {
            this.shooterButton = false;
        }
        if (shooterOn > 0) {
            if (!shooterMode) {
                shooterModeNum = 73.6;
            }

            shooterPowerLeft = ((-402.6 + Math.sqrt((Math.pow(402.6, 2)) + (-4.0) * (-192.0) * (-110.3 - shooterModeNum))) / (2.0 * (-192.0)));
            shooterPowerRight = ((0.2) / 204.6) * ((Math.hypot((Constants.FIELD_SIZE_CM_X - (2.5*Constants.TILE_SIZE_CM))-Robot.getRobotX(), (Constants.FIELD_SIZE_CM_Y)-Robot.getRobotY())) - 152.4);
            shooterPower = shooterPowerLeft + shooterPowerRight;
            shooter.setPower(shooterPower);

            //Replace later to setVelocity
           /* shooterPower = (shooterPowerLeft + shooterPowerRight)*((1620.0 / 60.0) * 103.6);;
            shooter.setVelocity(shooterPower);
            */

            shooterModeNum = 88.9;
        } else if (shooterOn < 0) {
            shooter.setPower(0);
        }

        if (hopperButton) {
            if(!this.hopperButton){
                this.hopperButton = true;
                hopperBeforeTime = System.currentTimeMillis();
            }
            if ((System.currentTimeMillis() - hopperBeforeTime) >= 200) {
                hopperOut = !hopperOut;
                hopperBeforeTime = System.currentTimeMillis();
            }
            if(hopperOut) {
                hopper.setPosition(1.0);
            }
            if (!hopperOut){
                hopper.setPosition(0);
            }
        }
        else {
            hopper.setPosition(0);
        }
        if(!hopperButton && this.hopperButton){
            this.hopperButton = false;
        }
    }
    public void shoot(){

    }
    @Override
    public void stop() {
    }
}