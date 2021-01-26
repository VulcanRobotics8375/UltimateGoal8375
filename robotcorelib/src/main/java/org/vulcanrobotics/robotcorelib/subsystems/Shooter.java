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
    private DcMotorEx shooter_one;
    private DcMotorEx shooter_two;
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
    private boolean powerShotOn = false;
    private double shooterModeNum = 88.9;
    private double shooterPowerLeft;
    private double shooterPowerRight;
    private double shooterHighPower = 0.895;
    private double shooterLowPower = 0.82;
    private double powerShotPower = 0.7;
    private float shooterHighButton;
    private float shooterLowButton;
    private int shooterOn = -1;
    private ElapsedTime servoTimer = new ElapsedTime();

    public Shooter() {}

    @Override
    public void init() {
        shooter_one = (DcMotorEx) hardwareMap.dcMotor.get("shooter_one");
        shooter_two = (DcMotorEx) hardwareMap.dcMotor.get("shooter_two");
        hopper = hardwareMap.servo.get("hopper");
        shooter_one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients coefficients = shooter_one.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_one.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10.0, 3.0, 0.0, 12.0));
        shooter_two.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10.0, 3.0, 0.0, 12.0));
        shooter_one.setDirection((DcMotor.Direction.FORWARD));
        shooter_two.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter_one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter_two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servoTimer.reset();
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
            shooter_one.setPower(shooterPower);
            telemetry.addData("shooter power", shooter_one.getPower());
        }
        else if(shooterHighButton > 0){
//            if(powerShotOn) {
//                powerShotOn = false;
//                shooter_two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
            shooter_one.setPower(shooterHighPower);
            shooter_two.setPower(shooterHighPower);
        }
        else if(shooterLowButton > 0){
            shooter_one.setPower(shooterLowPower);
            shooter_two.setPower(shooterLowPower);
        }
        else if(powerShotButton) {
//            if(!powerShotOn) {
//                powerShotOn = true;
//                shooter_two.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
            shooter_one.setPower(powerShotPower);
            shooter_two.setPower(powerShotPower);
        }

        else {
            shooter_one.setPower(0);
            shooter_two.setPower(0);
            pidRunning = false;
        }

//        telemetry.addData("p", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
//        telemetry.addData("i", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i);
//        telemetry.addData("d", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d);
//        telemetry.addData("f", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);

        if (hopperButton) {

            if(!this.hopperButton){
                this.hopperButton = true;
                hopperOut = true;
//                hopperBeforeTime = System.currentTimeMillis();
                servoTimer.reset();
            }

            if ((servoTimer.time(TimeUnit.MILLISECONDS)) >= 200) {
                hopperOut = !hopperOut;
//                hopperBeforeTime = System.currentTimeMillis();
                servoTimer.reset();
            }

            if(hopperOut) {
                hopper.setPosition(0.15);
            }

            if (!hopperOut){
                hopper.setPosition(0);
            }
        }

        else {
            hopper.setPosition(0);
//            servoTimer.reset();
        }

        if(!hopperButton && this.hopperButton){
            this.hopperButton = false;
        }



    }

    public void testShooterVelocity(boolean shooterOn) {
        double power = 0.84;
        double velocity = power * (1620.0 * 103.6 / 60.0);
        if(shooterOn) {
            shooter_one.setVelocity(velocity);
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
        double currentSpeed = (shooter_one.getCurrentPosition() - lastPosition) / (time / 1000.0);
        double error = (targetSpeed - currentSpeed) / (conversion);
        integral += ((error + lastError) / 2) / (time / 1000.0);
        double derivative = (error - lastError) / (time / 1000.0);

        double kp = 1;
        double ki = 1;
        double kd = 1;
        double output = (kp * error) + (ki * integral) + (kd * derivative);

        shooter_one.setPower(output);

        lastError = error;
        lastPosition = shooter_one.getCurrentPosition();


    }

    public void setPIDFCoefficients(PIDFCoefficients coefficients) {
        if(shooter_one != null) {
            shooter_one.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(coefficients.p, coefficients.i, coefficients.d, shooter_one.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f));
        }
    }

    public void setHopperPosition(double position) {
        hopper.setPosition(position);
    }

    public void setShooterPower(double power) {
        shooter_one.setPower(power);
    }

    public void shoot(){

    }
    @Override
    public void stop() {
    }
}