package org.vulcanrobotics.robotcorelib.subsystem;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.vulcanrobotics.robotcorelib.dashboard.Dashboard;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.DashboardMotor;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public class Drivetrain extends Subsystem {

    private DcMotor fl, fr, bl, br;
    private BNO055IMU imu;

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("front_left");
        fr = hardwareMap.dcMotor.get("front_right");
        bl = hardwareMap.dcMotor.get("back_left");
        br = hardwareMap.dcMotor.get("back_right");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

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

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    }


    public void fieldCentricMove(double x, double y, double turn) {

        double power = Math.hypot(x, y);

        double rx = Math.sin(Robot.getRobotAngleRad() - (Math.PI / 4));
        double lx = Math.cos(Robot.getRobotAngleRad() + (Math.PI / 4));

        double fl = lx + turn;
        double fr = rx - turn;
        double bl = rx + turn;
        double br = lx - turn;

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

    public void mecanumDrive(double forward, double turn, double strafe) {

        double vd = Math.hypot(forward, strafe);
        double theta = Math.atan2(forward, strafe) - (Math.PI / 4);
        double turnPower = turn;
        double multiplier = 1;

        double[] v = {
                vd * Math.sin(theta) + turnPower,
                vd * Math.cos(theta) - turnPower,
                vd * Math.cos(theta) + turnPower,
                vd * Math.sin(theta) - turnPower
        };

        double[] motorOut = {
                multiplier * (v[0] / 1.07) * ((0.62 * Math.pow(v[0], 2)) + 0.45),
                multiplier * (v[1] / 1.07) * ((0.62 * Math.pow(v[1], 2)) + 0.45),
                multiplier * (v[2] / 1.07) * ((0.62 * Math.pow(v[2], 2)) + 0.45),
                multiplier * (v[3] / 1.07) * ((0.62 * Math.pow(v[3], 2)) + 0.45)
        };

        fr.setPower(motorOut[0]);
        fl.setPower(motorOut[1]);
        br.setPower(motorOut[2]);
        bl.setPower(motorOut[3]);


    }

    @Override
    public void stop() {
        //set powers to zero, even though the rev hub will probs do it for you
        move(0, 0);
    }
}
