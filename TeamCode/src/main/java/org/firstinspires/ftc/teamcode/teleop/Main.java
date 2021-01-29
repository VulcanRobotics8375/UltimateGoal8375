package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.Mecanum;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "main", group = "main")
public class Main extends TeleOpPipeline {

    boolean debug = true;

    private ElapsedTime timer = new ElapsedTime();
    private double time;

    public void runOpMode() {

        teleopInit();
        Robot.loadRobotPosition();

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();
        telemetry.addLine("starting");
        telemetry.update();

        Robot.startOdometryThread();

        Mecanum motionProfile = (Mecanum) Robot.motionProfile;
        timer.reset();

        while (opModeIsActive()) {
            timer.reset();

            int shoot;
            if(gamepad1.a) {
                shoot = 1;
            } else if(gamepad1.b || gamepad1.y || gamepad1.x) {
                shoot = 2;
            } else {
                shoot = 0;
            }

            subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.a, gamepad1.b, gamepad1.y, gamepad1.x, gamepad1.left_bumper, gamepad1.right_bumper);
            subsystems.intake.run(gamepad2.b, gamepad2.a, gamepad2.right_bumper);
            subsystems.shooter.run(gamepad2.left_bumper, gamepad2.right_bumper, shoot, gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.dpad_down);
//            subsystems.wobbleGrabber.run(gamepad2.x,gamepad2.y, -gamepad2.left_stick_y * 0.5);

//            motionProfile.update();
            if(gamepad1.left_trigger > 0) {
                Robot.setRobotPos(new Point(Robot.getRobotX(), 21.6));
            }
            if(gamepad1.right_trigger  > 0) {
                Robot.setRobotPos(new Point(21.6, Robot.getRobotY()));
            }

            //TODO disable debug for competition code
            if(debug) {
                debug();
            }

//            telemetry.addData("left", motionProfile.getLeft().getPosition());
//            telemetry.addData("right", motionProfile.getRight().getPosition());
//            telemetry.addData("horizontal", motionProfile.getHorizontal().getPosition());
            telemetry.addData("loop time", time);
            telemetry.update();
            time = timer.milliseconds();
            timer.reset();
        }

    }

    private void debug() {
        telemetry.addData("robot x", Robot.getRobotX());
        telemetry.addData("robot y", Robot.getRobotY());
        telemetry.addData("robot angle", Robot.getRobotAngleDeg());
//        telemetry.addData("zAngle", subsystems.drivetrain.getZAngle());
    }

}
