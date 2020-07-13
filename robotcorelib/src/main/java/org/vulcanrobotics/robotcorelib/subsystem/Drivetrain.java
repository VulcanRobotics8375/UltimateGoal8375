package org.vulcanrobotics.robotcorelib.subsystem;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.vulcanrobotics.robotcorelib.Dashboard.Hardware.DashboardMotor;
import org.vulcanrobotics.robotcorelib.Dashboard.Hardware.Sensor;

public class Drivetrain extends Subsystem {

    private DashboardMotor fl, fr, bl, br;
    private BNO055IMU imu;

    @Override
    public void init() {
        fl = new DashboardMotor(hardwareMap.dcMotor.get("front_left"), 0);
        fr = new DashboardMotor(hardwareMap.dcMotor.get("front_right"), 1);
        bl = new DashboardMotor(hardwareMap.dcMotor.get("back_left"), 2);
        br = new DashboardMotor(hardwareMap.dcMotor.get("back_right"), 3);

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

    public void move(double forward, double turn) {
        fl.setPower(forward - turn);
        fr.setPower(forward + turn);
        bl.setPower(forward - turn);
        br.setPower(forward + turn);
    }

    public double getZAngle() {
      return AngleUnit.DEGREES.normalize(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

    @Override
    public void stop() {

    }
}
