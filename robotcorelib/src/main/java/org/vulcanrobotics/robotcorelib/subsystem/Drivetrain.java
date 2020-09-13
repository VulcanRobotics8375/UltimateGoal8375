package org.vulcanrobotics.robotcorelib.subsystem;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.vulcanrobotics.robotcorelib.dashboard.Dashboard;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.DashboardMotor;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public class Drivetrain extends Subsystem {

    private DashboardMotor fl, fr, bl, br;
    private BNO055IMU imu;

    @Override
    public void init() {
        fl = new DashboardMotor(hardwareMap.dcMotor.get("front_left"));
        fr = new DashboardMotor(hardwareMap.dcMotor.get("front_right"));
        bl = new DashboardMotor(hardwareMap.dcMotor.get("back_left"));
        br = new DashboardMotor(hardwareMap.dcMotor.get("back_right"));

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

        fl.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public DashboardMotor getFrontLeft() {
        return fl;
    }

    public DashboardMotor getBackLeft() {
        return bl;
    }

    public DashboardMotor getFrontRight() {
        return fr;
    }

    public DashboardMotor getBackRight() {
        return br;
    }

    @Override
    public void stop() {
        //set powers to zero, even though the rev hub will probs do it for you
        move(0, 0);
    }
}
