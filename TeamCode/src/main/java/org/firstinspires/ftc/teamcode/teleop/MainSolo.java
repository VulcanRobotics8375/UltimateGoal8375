package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.Mecanum;
import org.vulcanrobotics.robotcorelib.robot.Robot;

//@Disabled
@TeleOp(name = "solo", group = "main")
public class MainSolo extends TeleOpPipeline {

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

        boolean intakeOn = false;
        boolean shooterOn = false;
        boolean shooting = false;
        boolean intaking = false;

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

            if(gamepad1.left_bumper && !shooting) {
                shooting = true;
                shooterOn = !shooterOn;
            }

            if(!gamepad1.left_bumper && shooting) {
                shooting = false;
            }

            if(gamepad1.left_trigger > 0 && !intaking) {
                intaking = true;
                intakeOn = !intakeOn;
            }
            if(gamepad1.left_trigger == 0 && intaking) {
                intaking = false;
            }

            subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad2.a, gamepad2.b, gamepad2.y, gamepad2.x, gamepad1.dpad_left, gamepad1.dpad_right);
            subsystems.intake.run(intakeOn, gamepad1.y, gamepad2.right_bumper);
            subsystems.shooter.run(gamepad2.left_bumper, gamepad1.right_bumper, shoot, gamepad2.right_trigger, shooterOn ? 1 : 0, gamepad2.dpad_down);
            subsystems.wobbleGrabber.run(gamepad2.x,gamepad2.y, -gamepad2.left_stick_y * 0.5);

//            motionProfile.update();
//            if(gamepad1.left_trigger > 0) {
//                Robot.setRobotPos(new Point(Robot.getRobotX(), 21.6));
//            }
//            if(gamepad1.right_trigger  > 0) {
//                Robot.setRobotPos(new Point(21.6, Robot.getRobotY()));
//            }

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
