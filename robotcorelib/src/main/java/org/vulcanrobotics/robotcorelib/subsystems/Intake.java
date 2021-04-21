package org.vulcanrobotics.robotcorelib.subsystems;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.vulcanrobotics.robotcorelib.hardware.Rev2mDistanceSensorAutoCache;
import org.vulcanrobotics.robotcorelib.math.KalmanFilter;

public class Intake extends Subsystem {
    private DcMotor transfer, intake;
    private Rev2mDistanceSensor hopperSensor;
    private Rev2mDistanceSensorAutoCache fastSensor;
    private Servo ringBlocker, intakeDeploy;
    private boolean intakeButton, overrideBlocker, override = false;

    private HopperState hopperState = HopperState.ZERO_RINGS;
    private KalmanFilter filter = new KalmanFilter(0.2, 0.03, 90, 0.1);

    private ElapsedTime jamTimer = new ElapsedTime();


    @Override
    public void init() {
        transfer = hardwareMap.dcMotor.get("transfer_intake");
        intake = hardwareMap.dcMotor.get("roller_intake");
        hopperSensor = hardwareMap.get(Rev2mDistanceSensor.class, "hopper_sensor");
        ringBlocker = hardwareMap.servo.get("intake_blocker");
        intakeDeploy = hardwareMap.servo.get("intake_deploy");

        fastSensor = new Rev2mDistanceSensorAutoCache(hopperSensor, DistanceUnit.MM);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        //stop and reset encoder is important for odometry
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDeploy.setPosition(0.95);

    }

    //TODO add sensor code/ring counter for intake stage 1
    public void run(boolean intakeButton, boolean reverse, boolean transferOn, boolean overrideBlocker) {
        double transferSpeed = 1.0;
        double intakeSpeed = 1.0;

        if (intakeButton) {
//            transfer.setPower(transferSpeed);
            intake.setPower(intakeSpeed);
        } else if (reverse) {
//            transfer.setPower(transferSpeed);
            intake.setPower(-1.0 * intakeSpeed);
        } else {
            intake.setPower(0);
            transferSpeed = 0;
        }
        if(transferOn) {
            transferSpeed = -1.0;
        }
        transfer.setPower(transferSpeed);

        double hopperSensorRaw = fastSensor.getCachedDistance();
//        double hopperSensorRaw = hopperSensor.getDistance(DistanceUnit.MM);
        filter.run(hopperSensorRaw);
        double filterEstimate = filter.getEstimate();
//        telemetry.addData("sensor raw", hopperSensorRaw);
//        telemetry.addData("filter", filterEstimate);

        //TODO adjust bounds
        double oneRingBound = 90, twoRingBound = 70, threeRingBound = 50, obstructionBound = 20;

        if(filterEstimate > oneRingBound) {
            hopperState = HopperState.ZERO_RINGS;
        } else if(filterEstimate < oneRingBound && filterEstimate > twoRingBound) {
            hopperState = HopperState.ONE_RING;
        } else if(filterEstimate < twoRingBound && filterEstimate > threeRingBound) {
            hopperState = HopperState.TWO_RINGS;
        } else if(filterEstimate < threeRingBound && filterEstimate > obstructionBound) {
            hopperState = HopperState.THREE_RINGS;
        } else {
            hopperState = HopperState.SENSOR_OBSTRUCTED;
        }

//        telemetry.addData("sensor", hopperSensorRaw);
//        telemetry.addData("filter", filterEstimate);
//        telemetry.addData("hopper state", hopperState.ringNum);

        if(overrideBlocker && !this.overrideBlocker) {
            override = !override;
            this.overrideBlocker = true;
        }
        if(!overrideBlocker && this.overrideBlocker) {
            this.overrideBlocker = false;
        }

        if(!override) {
            if(reverse) {
                intakeDeploy.setPosition(0.8);
            } else {
                intakeDeploy.setPosition(0.7);
            }
            if (hopperState == HopperState.THREE_RINGS) {
                ringBlocker.setPosition(0.45);
            } else {
                ringBlocker.setPosition(0.25);
            }
        } else {
            ringBlocker.setPosition(0.9);
            intakeDeploy.setPosition(0.9);
        }
    }

    //TODO placeholder, pls remove
    public void run(boolean a, boolean b, boolean c) {}

    double lastTransferPos;
    private boolean isTransferJammed() {

        double transferPos = transfer.getCurrentPosition();
        double transferVelocity = Math.abs(transferPos) - Math.abs(lastTransferPos);

        //TODO remove telemetry
//        telemetry.addData("transfer velocity", transferVelocity);
        //tune these
        double transferSpeedJamThreshold = 20;
        double transferJamTimeout = 250;
        if(transferVelocity < transferSpeedJamThreshold && jamTimer.milliseconds() >= transferJamTimeout) {
            jamTimer.reset();
            lastTransferPos = transferPos;
            return true;
        }
        lastTransferPos = transferPos;
        return false;
    }


    public DcMotor getTransfer() {
        return transfer;
    }

    public DcMotor getIntake() {
        return intake;
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void setTransferPower(double power) {
        transfer.setPower(power);
    }

    public void setDeployPosition(double pos) {
        intakeDeploy.setPosition(pos);
    }

    public HopperState getHopperState() {
        return hopperState;
    }

    @Override
    public void stop() {
    }
}

enum HopperState {
    ZERO_RINGS(0),
    ONE_RING(1),
    TWO_RINGS(2),
    THREE_RINGS(3),
    SENSOR_OBSTRUCTED(-1);

    public int ringNum;

     HopperState(int ringNum) {
        this.ringNum = ringNum;
    }



}