package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.vulcanrobotics.robotcorelib.math.Functions;
import org.vulcanrobotics.robotcorelib.math.PID;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.Mecanum;
import org.vulcanrobotics.robotcorelib.robot.Robot;
import static org.vulcanrobotics.robotcorelib.framework.Constants.*;

public class Drivetrain extends Subsystem {

    private DcMotor fl, fr, bl, br;
    private BNO055IMU imu;

    private PID turnPid = new PID(2.2, 0.8, -2.7, 0.1, 0.1, -0.5, 0.5);

    private boolean doingAutonomousTask;
    private double variableOffset = 0;
    private boolean leftOffsetButton, rightOffsetButton;
    private boolean aimed = false;
    private boolean powerShotSequence = false;
    private PowerShot powerShot = PowerShot.NONE;

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("front_left");
        fr = hardwareMap.dcMotor.get("front_right");
        bl = hardwareMap.dcMotor.get("back_left");
        br = hardwareMap.dcMotor.get("back_right");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        variableOffset = 2;

        setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Robot.setResetPosition(new Point(21.6, 21.6));

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        if(imu.initialize(parameters)) {
            while (!imu.isGyroCalibrated()) {}
        } else {
            imu.initialize(parameters);
        }
    }

    public void autoInit() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turnPid.setKp(1.8);
//        turnPid.setKi(0.0);
//        turnPid.setKd(1.8);
    }

    public void move(double forward, double turn) {
        fl.setPower(forward - turn);
        fr.setPower(forward + turn);
        bl.setPower(forward - turn);
        br.setPower(forward + turn);
    }

    public void setPowers(double fl, double fr, double bl, double br) {
       this.fl.setPower(fl);
       this.fr.setPower(fr);
       this.bl.setPower(bl);
       this.br.setPower(br);
    }

    public void setPowers(double[] powers) {
        setPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    double[] lastPowers = new double[4];
    public void mecanumDrive(double forward, double turn, double strafe, boolean highGoal, boolean powerShotLeft, boolean powerShotCenter, boolean powerShotRight, boolean leftOffsetButton, boolean rightOffsetButton) {
        //handle/parse initial data for basic mecanum drive
//        if(forward == 0 && turn == 0 && strafe == 0) {
//            accelTimer.reset();
//        }

        turn = turn * 0.7;
        forward = curveLinearJoystick(forward);
        strafe = curveLinearJoystick(strafe);
        //scale inputs
        double magnitude = Math.abs(forward) + Math.abs(strafe) + Math.abs(turn);
        if(magnitude > 1) {
            forward *= 1.0 / magnitude;
            strafe *= 1.0 / magnitude;
            turn *= 1.0 / magnitude;

        }

        double vd = Math.hypot(strafe, forward);
        double theta = Math.atan2(forward, strafe) - (Math.PI / 4.0);
        double multiplier = Math.sqrt(2.0) * 1.0;
        double turnPower = turn;

        //check variable offset buttons, and change accordingly
        if(leftOffsetButton && !this.leftOffsetButton) {
            variableOffset -= 2.5;
            this.leftOffsetButton = true;
        }
        if(!leftOffsetButton && this.leftOffsetButton) {
            this.leftOffsetButton = false;
        }

        if(rightOffsetButton && !this.rightOffsetButton) {
            variableOffset += 2.5;
            this.rightOffsetButton = true;
        }
        if(!rightOffsetButton && this.rightOffsetButton) {
            this.rightOffsetButton = false;
        }

        //auto aim configuration
        double variableOffsetRad = Math.toRadians(variableOffset);
        Point target = new Point((1.5 * TILE_SIZE_CM), FIELD_SIZE_CM_Y);
        boolean aiming = false;
        if(powerShotLeft && !powerShotSequence) {
           powerShotSequence = true;
           powerShot = PowerShot.LEFT;
        } else if(!powerShotLeft && powerShotSequence) {
            powerShot = PowerShot.NONE;
            powerShotSequence = false;
        }

        if(highGoal) {
            target.setPoint(new Point((1.5 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
            aiming = true;
        }
        else if(powerShotSequence && powerShot == PowerShot.LEFT) {
            target.setPoint(new Point((2.1 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
            aiming = true;
        }
        else if(powerShotSequence && powerShot == PowerShot.CENTER) {
            target.setPoint(new Point((2.3 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
            aiming = true;
        }
        else if(powerShotSequence && powerShot == PowerShot.RIGHT) {
            target.setPoint(new Point((2.6 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
            aiming = true;
        }
        //auto aim calculation and determine drive style
        if(aiming) {
            doingAutonomousTask = true;
            double absoluteAngleToTarget = Math.atan2(target.x - Robot.getRobotX(), target.y - Robot.getRobotY());
            double currentAngle = Robot.getRobotAngleRad() > Math.PI ? Robot.getRobotAngleRad() - (2.0 * Math.PI) : Robot.getRobotAngleRad();
            //might have to subtract variable offset
            turnPid.run(absoluteAngleToTarget, currentAngle + variableOffsetRad);
            turnPower = turnPid.getOutput();
            double aimThresh = powerShot == PowerShot.NONE ? 0.035 : 0.015;
            aimed = Math.abs(absoluteAngleToTarget - currentAngle - variableOffsetRad) < aimThresh && Math.abs(turnPower) < 0.05;

            if(Robot.getComponents().shooter.shouldDriveStop()) {
                multiplier = 0;
            }
//            double feedforward = (Robot.getRobotXVelocity() / (135.0 * 4.0));
            fieldCentricMove(forward * multiplier, strafe * -1.0 * multiplier, (turnPower * -1.0));
        } else {
            double[] v = {
                    (vd * Math.cos(theta) + turnPower) * multiplier,
                    (vd * Math.sin(theta) - turnPower) * multiplier,
                    (vd * Math.sin(theta) + turnPower) * multiplier,
                    (vd * Math.cos(theta) - turnPower) * multiplier
            };

            setPowers(v);
            aimed = false;
            if(doingAutonomousTask) {
                turnPid.reset();
                doingAutonomousTask = false;
            }
        }

    }

    //right to left = 1 to 3, high goal = 0
    //invalid id defaults to aim at high goal
    public void aim(int shotId, double offset, double errorThresh) {
        Point target = new Point();
        if(shotId == 0) {
            target.setPoint(new Point((1.2 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
        }
        else if(shotId == 1) {
            target.setPoint(new Point((1.75 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
        }
        else if(shotId == 2) {
            target.setPoint(new Point((2.1 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
        }
        else if(shotId == 3) {
            target.setPoint(new Point((2.35 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
        }
        else {
            target.setPoint(new Point((1.5 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
        }

        //this is not the all angles solution, but it works fine since the goal is always in front of us
        double absoluteAngleToTarget = Math.atan2(target.x - Robot.getRobotX(), target.y - Robot.getRobotY());
        double currentAngle = Robot.getRobotAngleRad() > Math.PI ? Robot.getRobotAngleRad() - (2.0 * Math.PI) : Robot.getRobotAngleRad();
        //might have to subtract variable offset
        turnPid.run(absoluteAngleToTarget, currentAngle + offset);
        double error = absoluteAngleToTarget - currentAngle - offset;
        run(0, turnPid.getOutput() * -1.0);
        if(Math.abs(error) < errorThresh && Math.abs(turnPid.getOutput()) < 0.02) {
            aimed = true;
        } else {
            aimed = false;
        }

    }

    public void resetAiming() {
        aimed = false;
        turnPid.reset();
    }

    public void resetPidControllers() {
        turnPid.reset();
    }

    public void setAngleOffset(double offset) {
        variableOffset = offset;
    }


    private boolean pidUp, pidDown, pidSwitch;
    private int pidConstant = 0;
    public void tunePID(boolean up, boolean down, boolean constantSwitch) {

        double constant = 0;

        if(constantSwitch && !this.pidSwitch) {
            if(pidConstant == 2) {
                pidConstant = 0;
            } else {
                pidConstant++;
            }
            this.pidSwitch = true;
        }
        if(!constantSwitch && this.pidSwitch) {
            this.pidSwitch = false;
        }



        if(up && !this.pidUp) {
            constant += 0.1;
            this.pidUp = true;
        }
        if(!up && pidUp) {
            this.pidUp = false;
        }

        if(down && !this.pidDown) {
            constant -= 0.1;
            this.pidDown = true;
        }
        if(!down && this.pidDown) {
            this.pidDown = false;
        }

        if(pidConstant == 0) {
            constant += turnPid.getKp();
            turnPid.setKp(constant);
        }
        else if(pidConstant == 1) {
            constant += turnPid.getKi();
            turnPid.setKi(constant);
        }
        else if(pidConstant == 2) {
            constant += turnPid.getKd();
            turnPid.setKd(constant);
        }

        telemetry.addData("Kp", turnPid.getKp());
        telemetry.addData("Ki", turnPid.getKi());
        telemetry.addData("Kd", turnPid.getKd());

    }

    private double curveLinearJoystick(double input) {
        return (input / 1.07) * ((0.62 * Math.pow(input, 2)) + 0.45);
    }

    private double curveLinearJoystickCubic(double input) {
        return Math.pow(input, 3);
    }

    private double calculateMecanumGain(double angle) {
        int quadrant = 0;
        if(angle <= Math.PI / 2 && angle >= 0) {
            quadrant = 1;
        }
        else if(angle <= Math.PI && angle > Math.PI / 2) {
            quadrant = 3;
        }
        else if(angle >= -Math.PI && angle < -Math.PI / 2) {
            quadrant = 5;
        }
        else if(angle >= -Math.PI / 2 && angle < 0) {
            quadrant = 7;
        }
        double relativeAngle;
        if(angle >= 0) {
            relativeAngle = Math.abs(angle - (quadrant * Math.PI / 4));
        }
        else {
            relativeAngle = Math.abs(Math.abs(angle - (quadrant * Math.PI / 4)) - 2.0 * Math.PI);
        }

        double m = ((2 / Math.sqrt(2)) - 1) / (Math.PI / 4);
        return (m * relativeAngle) + 1;
    }

    //TODO test input scaling
    public void fieldCentricMove(double x, double y, double turn) {

//        y *= -1.0;
        //input scaling
//        double magnitude = Math.abs(x) + Math.abs(y) + Math.abs(turn);
//        if(magnitude > 1.0) {
//            x *= 1.0 / magnitude;
//            y *= 1.0 / magnitude;
//            turn *= 1.0 / magnitude;
//        }

        double power = Math.hypot(x, y);
        double theta = Robot.getRobotAngleRad() - Math.atan2(y, x);

        double rx = (Math.sin(theta + (Math.PI / 4))) * power;
        double lx = (Math.sin(theta - (Math.PI / 4))) * power;

        double fl = lx - turn;
        double fr = rx + turn;
        double bl = rx - turn;
        double br = lx + turn;

        setPowers(fl, fr, bl, br);

    }

    //TODO add PID
    //^^ easier said than done, bc PID may change based on batt voltage, we are running into the same problem as before
    //might not be an issue tho, our turning has been pretty consistent and we are using PID for that
    //and that turn PID is barely tuned anyway, it might just be running PD or P
    public void driveWithConstantVelocity(Point targetVelocity, double angularVelocity) {
        double errorX = targetVelocity.x - Robot.getRobotVelocity().x;
        double errorY = targetVelocity.y - Robot.getRobotVelocity().y;

        fieldCentricMove(errorX, errorY, angularVelocity);

    }

    @Deprecated
    //this isn't actually deprecated since we need it for backend/calibration,
    // but I never want to see it be used in any main code. ever.
    public double getZAngle() {
      return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public DcMotor getFrontLeft() {
        return fl;
    }

    public DcMotor getBackLeft() {
        return bl;
    }

    public DcMotor getFrontRight() {
        return fr;
    }

    public DcMotor getBackRight() {
        return br;
    }

    public boolean isAimed() {
        return aimed;
    }

//    public boolean isAiming()

    public void setDrivetrainMode(DcMotor.RunMode runMode) {
        fl.setMode(runMode);
        fr.setMode(runMode);
        bl.setMode(runMode);
        br.setMode(runMode);
    }

    public PID getTurnPid() {
        return turnPid;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior powerBehavior) {
        fl.setZeroPowerBehavior(powerBehavior);
        fr.setZeroPowerBehavior(powerBehavior);
        bl.setZeroPowerBehavior(powerBehavior);
        br.setZeroPowerBehavior(powerBehavior);
    }

    public void setVariableOffset(double offset) {
        variableOffset = offset;
    }

    public void run(double forward, double turn) {
        setPowers(forward - turn, forward + turn, forward - turn, forward + turn);
    }

    public void setPowerShot(PowerShot powerShot) {
        this.powerShot = powerShot;
    }

    public PowerShot getPowerShot() {
        return powerShot;
    }

    public boolean isDoingAutonomousTask() {
        return doingAutonomousTask;
    }

    @Override
    public void stop() {
        //set powers to zero, even though the rev hub will probs do it for you
        move(0, 0);
    }
}
