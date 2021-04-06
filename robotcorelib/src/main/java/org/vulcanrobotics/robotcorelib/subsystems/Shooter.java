package org.vulcanrobotics.robotcorelib.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import java.util.concurrent.TimeUnit;

public class Shooter extends Subsystem {
    private DcMotorEx shooter_one, shooter_two;
    private Servo hopper;

    private boolean hopperButton, hopperOut, powerShotButton = false, shooting = false, shootTrigger = false, drivetrainStopped, powerShotTrigger = false;
    public double shooterPower, powerShotMode = -1;
    double lastVelocity;

    private ElapsedTime servoTimer = new ElapsedTime();

    public Shooter() {}

    @Override
    public void init() {
        shooter_one = (DcMotorEx) hardwareMap.dcMotor.get("shooter_one");
        shooter_two = (DcMotorEx) hardwareMap.dcMotor.get("shooter_two");
        hopper = hardwareMap.servo.get("hopper");
        shooter_one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter_one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter_two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_two.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(13, 0.26, 0.0, 13.0));
        shooter_two.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter_one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter_two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servoTimer.reset();
        drivetrainStopped = false;
        shooting = false;
    }

    public void run(boolean shooterButton, boolean hopperButton, boolean powerShotButton, boolean robotMove) {

        boolean initialHopper = hopperButton;
        if (shooterButton && !shootTrigger) {
            shootTrigger = true;
            shooting = !shooting;
            powerShotTrigger = false;
        }
        if(powerShotButton && !this.powerShotButton){
            this.powerShotButton = true;
            powerShotTrigger = !powerShotTrigger;
            shooting = false;
        }

        if(!shooterButton && shootTrigger) {
            shootTrigger = false;
        }
        if(!powerShotButton && this.powerShotButton) {
            this.powerShotButton = false;
        }

        if(shooting) {
            shooter_two.setPower(0.805);
        } else if(powerShotTrigger) {
            shooter_two.setPower(0.675);
        } else {
            shooter_two.setPower(0);
        }


        double velocity = Math.hypot(Robot.getRobotXVelocity(), Robot.getRobotYVelocity());
//        telemetry.addData("accel", velocity - lastVelocity);
        if(!initialHopper) {
            if (velocity > 1 || Robot.getRobotY() > 215 || robotMove) {
                hopperButton = false;
            } else if (Robot.getComponents().intake.getHopperState() != HopperState.ZERO_RINGS && shooting && Robot.getComponents().drivetrain.isAimed() && Math.hypot(Robot.getRobotXVelocity(), Robot.getRobotYVelocity()) < 1 && velocity - lastVelocity < 1) {
                hopperButton = true;
            } else
                hopperButton = powerShotTrigger && Robot.getComponents().drivetrain.isAimed() && Robot.getComponents().drivetrain.getPowerShot() != PowerShot.NONE;
        }

        if (hopperButton) {
            if (!this.hopperButton) {
                this.hopperButton = true;
                hopperOut = true;
                servoTimer.reset();
            }
            if ((servoTimer.time(TimeUnit.MILLISECONDS)) >= 100) {
                hopperOut = !hopperOut;
                servoTimer.reset();
            }
            if (hopperOut) {
//                drivetrainStopped = true;
                hopper.setPosition(0.2);
                if(powerShotTrigger) {
                    Robot.getComponents().drivetrain.getTurnPid().reset();
                    PowerShot powerShot = Robot.getComponents().drivetrain.getPowerShot();
                    if(powerShot == PowerShot.LEFT) {
                        Robot.getComponents().drivetrain.setPowerShot(PowerShot.CENTER);
                    } else if(powerShot == PowerShot.CENTER) {
                        Robot.getComponents().drivetrain.setPowerShot(PowerShot.RIGHT);
                    } else {
                        Robot.getComponents().drivetrain.setPowerShot(PowerShot.NONE);
                    }
                }
            }
            else {
//                drivetrainStopped = false;
                hopper.setPosition(0);

            }
        } else {
            drivetrainStopped = false;

            hopper.setPosition(0);
        }

        if (!hopperButton && this.hopperButton) {
            this.hopperButton = false;
        }
        lastVelocity = velocity;
    }

    public void setPowers ( double power){
        shooter_two.setPower(power);
    }

    public DcMotor getShooterOne () {
        return shooter_one;
    }

    public DcMotor getShooterTwo () {
        return shooter_two;
    }

    public void setHopperPosition ( double position){
        hopper.setPosition(position);
    }

    public void setPowerShotPosition ( double position){
    }

    public void setShooterPower ( double power){
        shooter_one.setPower(power);
    }

    public boolean shouldDriveStop() {
        return drivetrainStopped;
    }

    @Override
    public void stop() {
    }
}