package org.vulcanrobotics.robotcorelib.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.vulcanrobotics.robotcorelib.framework.Constants;
import static org.vulcanrobotics.robotcorelib.framework.Constants.*;
public class Intake extends Subsystem {
    private DcMotor transfer, intake;
    private boolean intakeButton;

    //TODO add intake sensor after its on the robot

    @Override
    public void init() {
        transfer = hardwareMap.dcMotor.get("transfer_intake");
        intake = hardwareMap.dcMotor.get("roller_intake");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    //TODO add sensor code/ring counter for intake stage 1
    public void run(boolean intakeButton, boolean reverse, boolean transferOn) {
        //yeet. Im not throwin away my shot
        if (intakeButton) {
            transfer.setPower(1);
            intake.setPower(1);
        }

        else if (reverse) {
            transfer.setPower(-1);
            intake.setPower(-1);
        }

        else {
            transfer.setPower(0);
            intake.setPower(0);
        }

        if(transferOn) {
            transfer.setPower(0.35);
        }

    }

    @Override
    public void stop() {

    }
}