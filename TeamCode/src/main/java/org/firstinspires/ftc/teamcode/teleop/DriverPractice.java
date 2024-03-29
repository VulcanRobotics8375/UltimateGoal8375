package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.Mecanum;
import org.vulcanrobotics.robotcorelib.robot.Robot;

@TeleOp(name = "-- DRIVER PRACTICE --", group = "main")
public class DriverPractice extends TeleOpPipeline {

    boolean debug = true;

    public void runOpMode() {

        dash = false;
        ip = "";
        teleopInit();

        telemetry.addLine("ready");
        telemetry.update();

//        Robot.loadRobotPosition();
        setStart(new Point(138.34, 21.6), 0);

        waitForStart();
        telemetry.addLine("starting");
        telemetry.update();

        Robot.startOdometryThread();

        Mecanum motionProfile = (Mecanum) Robot.motionProfile;

        while (opModeIsActive()) {

            int shoot;
            if(gamepad1.a) {
                shoot = 1;
            } else if(gamepad1.b || gamepad1.y || gamepad1.x) {
                shoot = 2;
            } else {
                shoot = 0;
            }

            subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.a, gamepad1.b, gamepad1.y, gamepad1.x, gamepad1.left_bumper, gamepad1.right_bumper);
//            subsystems.intake.run(gamepad2.b, gamepad2.a, gamepad2.right_bumper);
//            subsystems.shooter.run(gamepad2.left_bumper, gamepad2.right_bumper, shoot, gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.dpad_down);
            subsystems.wobbleGrabber.run(gamepad2.x,gamepad2.y, -gamepad2.left_stick_y * 0.5);

//            motionProfile.update();

            //TODO disable debug for competition code
            if(debug) {
                debug();
            }

//            telemetry.addData("left", motionProfile.getLeft().getPosition());
//            telemetry.addData("right", motionProfile.getRight().getPosition());
//            telemetry.addData("horizontal", motionProfile.getHorizontal().getPosition());

            telemetry.update();
        }

    }

    private void debug() {
        telemetry.addData("robot x", Robot.getRobotX());
        telemetry.addData("robot y", Robot.getRobotY());
        telemetry.addData("robot angle", Robot.getRobotAngleDeg());
//        telemetry.addData("zAngle", subsystems.drivetrain.getZAngle());
    }

}
