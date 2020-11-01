package org.vulcanrobotics.robotcorelib.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.vulcanrobotics.robotcorelib.framework.Constants;
import static org.vulcanrobotics.robotcorelib.framework.Constants.*;
public class Intake extends Subsystem {
    private DcMotor transfer, intake;
    private boolean intakeButton;
    private double intakePower = 1.0;

    @Override
    public void init() {
        transfer = hardwareMap.dcMotor.get("transfer_intake");
        intake = hardwareMap.dcMotor.get("roller_intake");
    }

    public void run(boolean intakeButton, boolean reverse) {
        //yeet. Im not throwin away my shot
        if (intakeButton) {
            transfer.setPower(1);
            intake.setPower(1);
        } else if (reverse) {

            transfer.setPower(-1);
            intake.setPower(-1);

        } else {
            transfer.setPower(0);
            intake.setPower(0);
        }
    }

    @Override
    public void stop() {

    }
}