package org.vulcanrobotics.robotcorelib.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.vulcanrobotics.robotcorelib.framework.Constants;
import static org.vulcanrobotics.robotcorelib.framework.Constants.*;
public class Intake extends Subsystem {
    private DcMotor transfer, intake;
    private boolean intakeButton;
    private double transferSpeed = 1.0;
    private double intakeSpeed = 1.0;

    private ElapsedTime jamTimer = new ElapsedTime();


    @Override
    public void init() {
        transfer = hardwareMap.dcMotor.get("transfer_intake");
        intake = hardwareMap.dcMotor.get("roller_intake");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //TODO add sensor code/ring counter for intake stage 1
    public void run(boolean intakeButton, boolean reverse, boolean transferOn) {
        //yeet. Im not throwin away my shot
        if (intakeButton) {
            transfer.setPower(transferSpeed);
            intake.setPower(intakeSpeed);
        }

        else if (reverse) {
            transfer.setPower(transferSpeed * -1.0);
            intake.setPower(-1.0 * intakeSpeed);
        }

        else {
            transfer.setPower(0);
            intake.setPower(0);
        }

        if(transferOn) {
            transfer.setPower(0.5);
        }

    }

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

    @Override
    public void stop() {

    }
}