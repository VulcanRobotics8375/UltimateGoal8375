package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    private PID turnPid = new PID(1.8, 0.1, 1, 2.0, 0.05, -0.3, 0.3);

    private boolean doingAutonomousTask;
    private boolean unlockedAim;
    private double unlockAimAngle = 0;
    private double variableOffset = 0;
    private boolean leftOffsetButton, rightOffsetButton;
    private boolean aimed = false;

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("front_left");
        fr = hardwareMap.dcMotor.get("front_right");
        bl = hardwareMap.dcMotor.get("back_left");
        br = hardwareMap.dcMotor.get("back_right");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        turnPid.setKp(1.8);
        turnPid.setKi(0.0);
        turnPid.setKd(1.8);
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
        //handle/parse initial data for basic mecanum drive
        forward = curveLinearJoystick(forward);
        strafe = curveLinearJoystick(strafe);
        turn = curveLinearJoystick(turn);
        double vd = Math.hypot(forward, strafe);
        double theta = Math.atan2(forward, strafe) - (Math.PI / 4);
        double multiplier = Math.sqrt(2.0);
        double turnPower = turn;

        //check variable offset buttons, and change accordingly
        if(leftOffsetButton && !this.leftOffsetButton) {
            if(powerShotRight) {
                variableOffset -= 4.5;
            }
            else {
                variableOffset -= 3;
            }
            this.leftOffsetButton = true;
        }
        if(!leftOffsetButton && this.leftOffsetButton) {
            this.leftOffsetButton = false;
        }

        if(rightOffsetButton && !this.rightOffsetButton) {
            if(powerShotRight) {
                variableOffset += 4.5;
            }
            else {
                variableOffset += 3;
            }
            this.rightOffsetButton = true;
        }
        if(!rightOffsetButton && this.rightOffsetButton) {
            this.rightOffsetButton = false;
        }

        //auto aim configuration
        double variableOffsetRad = Math.toRadians(variableOffset);
        Point target = new Point();
        boolean aiming = false;
        if(highGoal) {
            target.setPoint(new Point(FIELD_SIZE_CM_X - (1.5 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
            aiming = true;
        }
        else if(powerShotLeft) {
            target.setPoint(new Point(FIELD_SIZE_CM_X - (2.25 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
            aiming = true;
        }
        else if(powerShotCenter) {
            target.setPoint(new Point(FIELD_SIZE_CM_X - (2.5 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
            aiming = true;
        }
        else if(powerShotRight) {
            target.setPoint(new Point(FIELD_SIZE_CM_X - (2.75 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
            aiming = true;
        }
        //auto aim calculation and determine drive style
        if(aiming) {
            double absoluteAngleToTarget = Math.atan2(target.y - Robot.getRobotY(), target.x - Robot.getRobotX());
            doingAutonomousTask = true;
            double error = Functions.angleWrap(absoluteAngleToTarget - (Robot.getRobotAngleRad() * -1.0));
            //non pid code for testing
            turnPid.run(absoluteAngleToTarget, Functions.angleWrap(((Robot.getRobotAngleRad() * -1.0) + variableOffsetRad + SHOOTING_OFFSET_RAD)));
//            turnPower = (error + variableOffsetRad + SHOOTING_OFFSET_RAD) * turnPid.getKp();
            turnPower = turnPid.getOutput() * -1.0;
            fieldCentricMove(strafe, forward, turnPower);
        } else {
            //basic mecanum calculations
            double[] v = {
                    (vd * Math.cos(theta) + turnPower) * multiplier,
                    (vd * Math.sin(theta) - turnPower) * multiplier,
                    (vd * Math.sin(theta) + turnPower) * multiplier,
                    (vd * Math.cos(theta) - turnPower) * multiplier
            };
            if(doingAutonomousTask) {
                turnPid.reset();
                doingAutonomousTask = false;
            }
            setPowers(v);
        }
    }

    //right to left = 1 to 3, high goal = 0
    //invalid id defaults to aim at high goal
    public void aim(int shotId, double errorThresh) {
        Point target = new Point();
        if(shotId == 0) {
            target.setPoint(new Point(FIELD_SIZE_CM_X - (1.5 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
        }
        else if(shotId == 1) {
            target.setPoint(new Point(FIELD_SIZE_CM_X - (2.1 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
        }
        else if(shotId == 2) {
            target.setPoint(new Point(FIELD_SIZE_CM_X - (2.42 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
        }
        else if(shotId == 3) {
            target.setPoint(new Point(FIELD_SIZE_CM_X - (2.73 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
        }
        else {
            target.setPoint(new Point(FIELD_SIZE_CM_X - (1.5 * TILE_SIZE_CM), FIELD_SIZE_CM_Y));
        }

        double absoluteAngleToTarget = Math.atan2(target.y - Robot.getRobotY(), target.x - Robot.getRobotX());
        double error = absoluteAngleToTarget - Functions.angleWrap((Robot.getRobotAngleRad() * -1.0) + SHOOTING_OFFSET_RAD);
        turnPid.run(absoluteAngleToTarget, Functions.angleWrap(((Robot.getRobotAngleRad() * -1.0) + SHOOTING_OFFSET_RAD)));
        run(0, turnPid.getOutput() * -1.0);
        if(error < errorThresh) {
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

    public boolean isAimed() {
        return aimed;
    }

    public void setDrivetrainMode(DcMotor.RunMode runMode) {
        fl.setMode(runMode);
        fr.setMode(runMode);
        bl.setMode(runMode);
        br.setMode(runMode);
    }

    public void run(double forward, double turn) {
        setPowers(forward - turn, forward + turn, forward - turn, forward + turn);
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
