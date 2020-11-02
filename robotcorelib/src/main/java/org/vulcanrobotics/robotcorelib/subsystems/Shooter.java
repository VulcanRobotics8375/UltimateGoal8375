package org.vulcanrobotics.robotcorelib.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public class Shooter extends Subsystem {
    public DcMotorEx shooter;
    public Servo hopper;
    private boolean hopperButton;
    private double a = 402.6;
    private double b = 192.0;
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

    public Shooter() {}

    @Override
    public void init() {
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
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

            shooterPowerLeft = ((-a + Math.sqrt((Math.pow(a, 2)) + (-4.0) * (-b) * (-110.3 - shooterModeNum))) / (2.0 * (-b)));
            shooterPowerRight = ((0.12) / 204.6) * ((Math.hypot((Constants.FIELD_SIZE_CM_X - (2.5 * Constants.TILE_SIZE_CM)) - Robot.getRobotX(), (Constants.FIELD_SIZE_CM_Y) - Robot.getRobotY())) - 152.4);

            //Replace setVelocity equation
            //*((1620.0 / 60.0) * 103.6)
            shooterPower = (shooterPowerLeft + shooterPowerRight);
            ;
            shooter.setPower(shooterPower);
            shooterModeNum = 88.9;
        } else if (shooterOn < 0) {
            shooter.setPower(0);
        }


        if (hopperButton) {

            if(!this.hopperButton){
                this.hopperButton = true;
                hopperBeforeTime = System.currentTimeMillis();
            }

            if ((System.currentTimeMillis() - hopperBeforeTime) >= 450) {
                hopperOut = !hopperOut;
                hopperBeforeTime = System.currentTimeMillis();
            }

            if(hopperOut) {
                hopper.setPosition(.35;
            }

            if (!hopperOut){
                hopper.setPosition(.2);
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