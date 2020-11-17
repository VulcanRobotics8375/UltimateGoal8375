package org.vulcanrobotics.robotcorelib.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public class Shooter extends Subsystem {
    private DcMotorEx shooter;
    private Servo hopper;

    private boolean hopperButton;
    private double a = -417.9;
    private double b = 832;
    private double hopperBeforeTime;
    private boolean hopperOut;
    private boolean shooterButton;
    //lowest shooter power when closest is 0.8, highest shooter power when furthest is
    public double shooterPower;
    //on = goal, off = powershot
    private boolean shooterMode = true;
    private double shooterModeNum = 88.9;
    private double shooterPowerLeft;
    private double shooterPowerRight;
    private double shooterHighPower = 0.895;
    private double shooterLowPower = 0.83;
    private double powerShotPower = 0.77;
    private float shooterHighButton;
    private float shooterLowButton;
    private int shooterOn = -1;

    public Shooter() {}

    @Override
    public void init() {
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        hopper = hardwareMap.servo.get("hopper");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        shooter.getMotorType().setMaxRPM(1620);
//        shooter.getMotorType().setGearing(3.7);
//        shooter.getMotorType().setTicksPerRev(103.6);
//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooter.setVelocityPIDFCoefficients(1.2, 0.12, 0, 11.7);
        shooter.setDirection((DcMotor.Direction.REVERSE));
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //TODO put the shooter power calculation in a separate method to clean up some stuff
    public void run(boolean shooterButton, boolean hopperButton, int shooterMode, float shooterHighButton, float shooterLowButton, boolean powerShotButton) {
        if(shooterMode == 1) {
            this.shooterMode = true;
        } else if(shooterMode == 2) {
            this.shooterMode = false;
        }

        if (shooterButton) {
            if(!this.shooterMode) {
                shooterModeNum = 73.6;
            }
            else {
                shooterModeNum = 88.9;
            }

            shooterPowerLeft = ((-b + Math.sqrt((Math.pow(b, 2)) + (-4.0) * (a) * (-313.7 - shooterModeNum))) / (2.0 * (a)));
            shooterPowerRight = ((0.14) / 204.6) * ((Math.hypot((Constants.FIELD_SIZE_CM_X - (2.5 * Constants.TILE_SIZE_CM)) - Robot.getRobotX(), (Constants.FIELD_SIZE_CM_Y) - Robot.getRobotY())) - 152.4);

            //Replace setVelocity equation
            //
            shooterPower = (shooterPowerLeft + shooterPowerRight) - 0.05;
            if(shooterPower < shooterLowPower){
                shooterPower = shooterLowPower;
            }
            if(shooterPower > shooterHighPower){
                shooterPower = shooterHighPower;
            }
            shooter.setPower(shooterPower);
            telemetry.addData("shooter power", shooter.getPower());
        }
        else if(shooterHighButton > 0){
            shooter.setPower(shooterHighPower);
        }
        else if(shooterLowButton > 0){
            shooter.setPower(shooterLowPower);
        }
        else if(powerShotButton) {
            shooter.setPower(powerShotPower);
        }

        else {
            shooter.setPower(0);
            pidRunning = false;
        }


        //copy paste this somehwere else, it doesn't need to stay in this file
/*
        if (shooterButton && !this.shooterButton) {
            shooterOn *= -1;
            this.shooterButton = true;
        }

        if (!shooterButton && this.shooterButton) {
            this.shooterButton = false;
        }

        if(shooterMode == 1) {
            this.shooterMode = true;
        }
        if (shooterMode == 2) {
            this.shooterMode = false;
        }

        if (shooterOn > 0) {

            if(!this.shooterMode) {
                shooterModeNum = 73.6;
            }
            else {
                shooterModeNum = 88.9;
            }

            shooterPowerLeft = ((-b + Math.sqrt((Math.pow(b, 2)) + (-4.0) * (a) * (-313.7 - shooterModeNum))) / (2.0 * (a)));
            shooterPowerRight = ((0.14) / 204.6) * ((Math.hypot((Constants.FIELD_SIZE_CM_X - (2.5 * Constants.TILE_SIZE_CM)) - Robot.getRobotX(), (Constants.FIELD_SIZE_CM_Y) - Robot.getRobotY())) - 152.4);

            //Replace setVelocity equation
            //
            shooterPower = (shooterPowerLeft + shooterPowerRight);
            if(shooterPower < 0.83){
                shooterPower = 0.83;
            }
            shooter.setPower(shooterPower);
            telemetry.addData("shooter power", shooter.getPower());

        } else if (shooterOn < 0) {
            shooter.setPower(0);
            pidRunning = false;
        }

 */


        if (hopperButton) {

            if(!this.hopperButton){
                this.hopperButton = true;
                hopperBeforeTime = System.currentTimeMillis();
            }

            if ((System.currentTimeMillis() - hopperBeforeTime) >= 500) {
                hopperOut = !hopperOut;
                hopperBeforeTime = System.currentTimeMillis();
            }

            if(hopperOut) {
                hopper.setPosition(.35);
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

    public void testShooterVelocity(boolean shooterOn) {
        double power = 0.84;
        double velocity = power * (1620.0 * 103.6 / 60.0);
        if(shooterOn) {
            shooter.setVelocity(velocity);
        }
    }

    private long lastPidTime;
    private boolean pidRunning;
    private double lastError, integral = 0, lastPosition;

    public void setShooterVelocity(double power) {
        if(!pidRunning) {
            lastPidTime = System.currentTimeMillis();
            integral = 0;
            lastError = 0;
            pidRunning = true;
        }
        //input from 0 to 1

        long time = System.currentTimeMillis() - lastPidTime;

        double conversion = ((1620.0 / 60.0) * 103.6);

        double targetSpeed = power * conversion;
        double currentSpeed = (shooter.getCurrentPosition() - lastPosition) / (time / 1000.0);
        double error = (targetSpeed - currentSpeed) / (conversion);
        integral += ((error + lastError) / 2) / (time / 1000.0);
        double derivative = (error - lastError) / (time / 1000.0);

        double kp = 1;
        double ki = 1;
        double kd = 1;
        double output = (kp * error) + (ki * integral) + (kd * derivative);

        shooter.setPower(output);

        lastError = error;
        lastPosition = shooter.getCurrentPosition();


    }

    public void setHopperPosition(double position) {
        hopper.setPosition(position);
    }

    public void setShooterPower(double power) {
        shooter.setPower(power);
    }

    public void shoot(){

    }
    @Override
    public void stop() {
    }
}