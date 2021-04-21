package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.vulcanrobotics.robotcorelib.drive.StandardTrackingWheelLocalizer;
import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline;
import org.vulcanrobotics.robotcorelib.math.PathPoint;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.Mecanum;
import org.vulcanrobotics.robotcorelib.motion.PurePursuit;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "main", group = "main")
public class Main extends TeleOpPipeline {

    protected boolean driverPractice = false;

    private ElapsedTime timer = new ElapsedTime();
    private double time;

    public void runOpMode() {
        StandardTrackingWheelLocalizer drive = new StandardTrackingWheelLocalizer(hardwareMap);
        //possible new startX is 105.5 because of X direction swap with rr code
        //also might be horizontal encoder direction so check that as well
        //load robot position from auto
        if(driverPractice) {
            drive.setPoseEstimate(new Pose2d(21.6, 108, 0.0));
        } else {
            Robot.loadRobotPosition();
            drive.setPoseEstimate(new Pose2d(Robot.getRobotY(), Robot.getRobotX(), Robot.getRobotAngleRad()));
        }

        teleopInit();
        Robot.loadRobotPosition();
//        gamepad1.setJoystickDeadzone(0.001f);

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();
        telemetry.addLine("starting");
        telemetry.update();

//        Robot.startOdometryThread();

//        Mecanum motionProfile = (Mecanum) Robot.motionProfile;
//        timer.reset();
        boolean reset = false;

        gamepad1.setJoystickDeadzone(0.1f);

        while (opModeIsActive()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            Pose2d velEstimate = drive.getPoseVelocity();
            Robot.setRobotPos(new Point(poseEstimate.getY(), poseEstimate.getX()));
            Robot.setRobotAngle(poseEstimate.getHeading());
            Robot.setRobotVelocity(new Point(velEstimate.getY(), velEstimate.getX()));
//            timer.reset();
            if(gamepad1.right_trigger != 0 && !reset) {
                drive.setPoseEstimate(new Pose2d(170, poseEstimate.getY(), poseEstimate.getHeading()));
                //tune this value
//                subsystems.drivetrain.setVariableOffset(2.5);
                reset = true;
            }
            if(reset && gamepad1.left_trigger == 0) {
                reset = false;
            }


            subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.a, gamepad1.b, gamepad1.y, gamepad1.x, gamepad1.left_bumper, gamepad1.right_bumper);
            subsystems.intake.run(gamepad2.b, gamepad2.a, gamepad2.dpad_down, gamepad2.dpad_up);
            subsystems.shooter.run(gamepad2.left_trigger > 0, gamepad2.right_bumper, gamepad2.left_bumper, gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0);
//            subsystems.wobbleGrabber.run(gamepad2.x, gamepad2.y, -gamepad2.left_stick_y * 0.5);
            subsystems.wobbleGrabber.setTurnPosition(0.65);



//            double TARGET_UPS = 100.0;
//            while(timer.milliseconds() < 1000.0 / TARGET_UPS) {
////               sleep(1);
//            }

//            subsystems.drivetrain.tunePID(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_right);
//            telemetry.addData("velocity", Math.hypot(Robot.getRobotXVelocity(), Robot.getRobotYVelocity()));
//            telemetry.addData("y pos", Robot.getRobotY());
            telemetry.addData("theta", Robot.getRobotAngleRad());
            telemetry.update();

//            time = timer.milliseconds();
        }

    }

    private void safeSleep(long ms, boolean stop) {
        if(!stop) {
            sleep(ms);
        }
    }

    private void debug() {
        telemetry.addData("robot x", Robot.getRobotX());
        telemetry.addData("robot y", Robot.getRobotY());
        telemetry.addData("robot angle", Robot.getRobotAngleRad());
//        telemetry.addData()

//        telemetry.update();
//        telemetry.addData("zAngle", subsystems.drivetrain.getZAngle());
    }

}
