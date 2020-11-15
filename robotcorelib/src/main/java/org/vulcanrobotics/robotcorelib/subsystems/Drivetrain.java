package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.vulcanrobotics.robotcorelib.math.Functions;
import org.vulcanrobotics.robotcorelib.math.PID;
import org.vulcanrobotics.robotcorelib.robot.Robot;
import static org.vulcanrobotics.robotcorelib.framework.Constants.*;

public class Drivetrain extends Subsystem {

    private DcMotor fl, fr, bl, br;
    private BNO055IMU imu;

    private PID turnPid = new PID(1.5, 0.01, 2);

    private boolean doingAutonomousTask;
    private double variableOffset = 0;
    private boolean leftOffsetButton, rightOffsetButton;

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("front_left");
        fr = hardwareMap.dcMotor.get("front_right");
        bl = hardwareMap.dcMotor.get("back_left");
        br = hardwareMap.dcMotor.get("back_right");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
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

    public void mecanumDrive(double forward, double turn, double strafe, boolean highGoal, boolean powerShotLeft, boolean powerShotCenter, boolean powerShotRight, boolean leftOffsetButton, boolean rightOffsetButton) {
        forward = curveLinearJoystick(forward);
        strafe = curveLinearJoystick(strafe);
        turn = curveLinearJoystick(turn);
        double vd = Math.hypot(forward, strafe);
        double theta = Math.atan2(forward, strafe) - (Math.PI / 4);
        double multiplier = 1.0;
        double turnPower = turn;

        if(leftOffsetButton && !this.leftOffsetButton) {
            variableOffset -= 2;
            this.leftOffsetButton = true;
        }
        if(!leftOffsetButton && this.leftOffsetButton) {
            this.leftOffsetButton = false;
        }

        if(rightOffsetButton && !this.rightOffsetButton) {
            variableOffset += 2;
            this.rightOffsetButton = true;
        }
        if(!rightOffsetButton && this.rightOffsetButton) {
            this.rightOffsetButton = false;
        }

        if(highGoal) {
            double absoluteAngleToTarget = Math.atan2(FIELD_SIZE_CM_Y - Robot.getRobotY(), (FIELD_SIZE_CM_X - (1.5*TILE_SIZE_CM)) - Robot.getRobotX());
            double distanceToTarget = Math.hypot((FIELD_SIZE_CM_X - (1.5*TILE_SIZE_CM)) - Robot.getRobotX(), (FIELD_SIZE_CM_Y - Robot.getRobotY()));

            double offset = ((((SHOOTING_OFFSET_MIN - SHOOTING_OFFSET_MAX) / 204.6) * (distanceToTarget - 152.4)) + SHOOTING_OFFSET_MAX) + SHOOTING_DEGREE_BIAS;

            doingAutonomousTask = true;
            double error = Functions.angleWrap(Math.toRadians(getZAngle() + offset));
            turnPid.run(absoluteAngleToTarget, error);
            turnPower = turnPid.getOutput();

        } else if(powerShotCenter || powerShotLeft || powerShotRight) {
            doingAutonomousTask = true;
            double absoluteAngleToTarget;
            double distanceToTarget;
            double offset;
            if(powerShotLeft) {
                absoluteAngleToTarget = Math.atan2(FIELD_SIZE_CM_Y - Robot.getRobotY(), (FIELD_SIZE_CM_X - (2.75 * TILE_SIZE_CM)) - Robot.getRobotX());
                distanceToTarget = Math.hypot((FIELD_SIZE_CM_X - (2.75*TILE_SIZE_CM)) - Robot.getRobotX(), (FIELD_SIZE_CM_Y - Robot.getRobotY()));

                offset = ((((SHOOTING_OFFSET_MIN - SHOOTING_OFFSET_MAX) / 204.6) * (distanceToTarget - 152.4)) + SHOOTING_OFFSET_MAX) + SHOOTING_DEGREE_BIAS + variableOffset;
            }
            else if(powerShotCenter) {
                absoluteAngleToTarget = Math.atan2(FIELD_SIZE_CM_Y - Robot.getRobotY(), (FIELD_SIZE_CM_X - (2.5 * TILE_SIZE_CM)) - Robot.getRobotX());
                distanceToTarget = Math.hypot((FIELD_SIZE_CM_X - (2.5*TILE_SIZE_CM)) - Robot.getRobotX(), (FIELD_SIZE_CM_Y - Robot.getRobotY()));

                offset = ((((SHOOTING_OFFSET_MIN - SHOOTING_OFFSET_MAX) / 204.6) * (distanceToTarget - 152.4)) + SHOOTING_OFFSET_MAX) + SHOOTING_DEGREE_BIAS + variableOffset;
            }
            else {
                absoluteAngleToTarget = Math.atan2(FIELD_SIZE_CM_Y - Robot.getRobotY(), (FIELD_SIZE_CM_X - (2.25 * TILE_SIZE_CM)) - Robot.getRobotX());
                distanceToTarget = Math.hypot((FIELD_SIZE_CM_X - (2.25*TILE_SIZE_CM)) - Robot.getRobotX(), (FIELD_SIZE_CM_Y - Robot.getRobotY()));

                offset = ((((SHOOTING_OFFSET_MIN - SHOOTING_OFFSET_MAX) / 204.6) * (distanceToTarget - 152.4)) + SHOOTING_OFFSET_MAX) + SHOOTING_DEGREE_BIAS + variableOffset;
            }

            double error = Math.toRadians(Robot.getRobotAngleDeg() + offset);
            turnPid.run(absoluteAngleToTarget + SHOOTING_DEGREE_BIAS, error);
            turnPower = turnPid.getOutput();

        }
        else {
            doingAutonomousTask = false;
            turnPid.reset();
        }

        double[] v = {
                vd * Math.sin(theta) - turnPower,
                vd * Math.cos(theta) + turnPower,
                vd * Math.cos(theta) - turnPower,
                vd * Math.sin(theta) + turnPower
        };

        double[] motorOut = {
                multiplier * v[0],
                multiplier * v[1],
                multiplier * v[2],
                multiplier * v[3]
        };

        fr.setPower(motorOut[0]);
        fl.setPower(motorOut[1]);
        br.setPower(motorOut[2]);
        bl.setPower(motorOut[3]);


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

    public void fieldCentricMove(double x, double y, double turn) {

        x *= -1.0;
        double power = Math.hypot(x, y);
        double theta = Math.atan2(y, x) - Robot.getRobotAngleRad();

        double rx = (Math.sin(theta + (Math.PI / 4))) * power;
        double lx = (Math.sin(theta - (Math.PI / 4))) * power;

        double fl = lx - turn;
        double fr = rx + turn;
        double bl = rx - turn;
        double br = lx + turn;

        setPowers(fl, fr, bl, br);

    }

    public double getZAngle() {
      return AngleUnit.DEGREES.normalize(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
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

    private void setDrivetrainMode(DcMotor.RunMode runMode) {
        fl.setMode(runMode);
        fr.setMode(runMode);
        bl.setMode(runMode);
        br.setMode(runMode);
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
