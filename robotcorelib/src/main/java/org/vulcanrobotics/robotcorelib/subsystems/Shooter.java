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
    private Servo powerShot;
    private boolean hopperButton, hopperOut, powerShotButton = false;
    public double shooterPower, powerShotMode = -1;
    private ElapsedTime servoTimer = new ElapsedTime();

    public Shooter() {}

    @Override
    public void init() {
        shooter_one = (DcMotorEx) hardwareMap.dcMotor.get("shooter_one");
        shooter_two = (DcMotorEx) hardwareMap.dcMotor.get("shooter_two");
        hopper = hardwareMap.servo.get("hopper");
        powerShot = hardwareMap.servo.get("powershot");
        shooter_one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter_one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter_two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_two.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10.0, 3.0, 0.0, 6.0));
        shooter_two.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter_one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter_two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servoTimer.reset();
    }

    //TODO put the shooter power calculation in a separate method to clean up some stuff
    public void run(boolean shooterButton, boolean hopperButton, boolean powerShotButton) {

        if (shooterButton) {
            shooterPower = .70;
            shooter_two.setPower(shooterPower);
         //   telemetry.addData("shooter power", shooter_two.getPower());
        } else {
            shooter_two.setPower(0);
        }

        if(powerShotButton && !this.powerShotButton) {
            powerShotMode *= -1;
            this.powerShotButton = true;
        }
        if(!powerShotButton && this.powerShotButton){
            this.powerShotButton = false;
        }

        if(powerShotMode > 0){
            powerShot.setPosition(0.65);
        }
        else if(powerShotMode < 0){
            powerShot.setPosition(0.035);
        }

        if (hopperButton) {
            if(!this.hopperButton){
                this.hopperButton = true;
                hopperOut = true;
                servoTimer.reset();
            }
            if ((servoTimer.time(TimeUnit.MILLISECONDS)) >= 200) {
                hopperOut = !hopperOut;
                servoTimer.reset();
            }
            if(hopperOut) {
                hopper.setPosition(0.2);
            }
            if (!hopperOut){
                hopper.setPosition(0);
            }
        } else {
            hopper.setPosition(0);
        }

        if(!hopperButton && this.hopperButton){
            this.hopperButton = false;
        }
    }

    public void setPowers(double power) {
        shooter_two.setPower(power);
    }

    public DcMotor getShooterOne() {
        return shooter_one;
    }

    public DcMotor getShooterTwo() {
        return shooter_two;
    }

    public void setHopperPosition(double position) {
        hopper.setPosition(position);
    }

    public void setPowerShotPosition(double position) {
        powerShot.setPosition(position);
    }

    public void setShooterPower(double power) {
        shooter_one.setPower(power);
    }

    @Override
    public void stop() {
    }
}