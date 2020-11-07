package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.Mecanum;
import org.vulcanrobotics.robotcorelib.robot.Robot;

@TeleOp(name = "main", group = "main")
public class Main extends TeleOpPipeline {

    boolean debug = true;

    public void runOpMode() {

        dash = false;
        ip = "";
        teleopInit();

        telemetry.addLine("ready");
        telemetry.update();

        setStart(new Point(138.34, 21.6), 0);

        waitForStart();
        telemetry.addLine("starting");
        telemetry.update();

        Robot.startOdometryThread();

        Mecanum motionProfile = (Mecanum) Robot.motionProfile;

        while (opModeIsActive()) {

            boolean shoot;
            if(gamepad1.a) {
                shoot = true;
            } else if(gamepad1.b || gamepad1.y || gamepad1.x) {
                shoot = false;
            } else {
                shoot = true;
            }

            subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.left_bumper, gamepad1.a, gamepad1.b, gamepad1.y, gamepad1.x);
            subsystems.intake.run(gamepad2.a, gamepad2.b);
            subsystems.shooter.run(gamepad2.right_bumper, gamepad2.left_bumper, shoot);

            //TODO disable debug for competition code
            if(debug) {
                debug();
            }

            telemetry.update();
        }

    }

    private void debug() {
        telemetry.addData("robot x", Robot.getRobotX());
        telemetry.addData("robot y", Robot.getRobotY());
        telemetry.addData("robot angle", Robot.getRobotAngleDeg());
//            telemetry.addData("left", motionProfile.getLeft().getPosition());
//            telemetry.addData("right", motionProfile.getRight().getPosition());
        telemetry.addData("zAngle", subsystems.drivetrain.getZAngle());
    }

}
