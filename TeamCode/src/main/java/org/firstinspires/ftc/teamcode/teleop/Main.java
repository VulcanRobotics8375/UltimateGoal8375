package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.motion.Mecanum;
import org.vulcanrobotics.robotcorelib.robot.Robot;

@TeleOp(name = "main", group = "main")
public class Main extends TeleOpPipeline {

    boolean debug = false;

    public void runOpMode() {

        dash = false;
        ip = "";
        teleopInit();

        telemetry.addLine("ready");
        telemetry.update();


        waitForStart();
        telemetry.addLine("starting");
        telemetry.update();

        Robot.startOdometryThread();

        Mecanum motionProfile = (Mecanum) Robot.motionProfile;

        while (opModeIsActive()) {

            boolean shoot = gamepad1.a;
            boolean powerShot = gamepad1.b;

            subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.left_bumper, shoot, powerShot);
            subsystems.intake.run(gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y);
            subsystems.shooter.run(gamepad2.right_bumper, gamepad2.left_bumper);

            //TODO disable debug for competition code
//            if(debug) {
//                debug();
//            }
            telemetry.addData("robot x", Robot.getRobotX());
            telemetry.addData("robot y", Robot.getRobotY());
            telemetry.addData("robot angle", Robot.getRobotAngleDeg());
//            telemetry.addData("left", motionProfile.getLeft().getPosition());
//            telemetry.addData("right", motionProfile.getRight().getPosition());
            telemetry.addData("horizontal", motionProfile.getHorizontal().getPosition());
            telemetry.addData("zAngle", subsystems.drivetrain.getZAngle());

            telemetry.update();
        }

    }

    private void debug() {
//        telemetry.addData("left position", motionProfile.getLeft().getPosition());
//        telemetry.addData("right position", motionProfile.getRight().getPosition());
//        telemetry.addData("strafe position", motionProfile.getHorizontal().getPosition());
//        telemetry.addData("robot position", Robot.getRobotX() + ", " + Robot.getRobotY());
    }

}
