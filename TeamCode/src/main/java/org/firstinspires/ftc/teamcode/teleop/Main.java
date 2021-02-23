package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    boolean debug = true;
    boolean autoRunning = false;
    volatile float joystickCancel;

    private ElapsedTime timer = new ElapsedTime();
    private double time;

    public void runOpMode() {

        teleopInit();
        Robot.loadRobotPosition();
        gamepad1.setJoystickDeadzone(0.001f);

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();
        telemetry.addLine("starting");
        telemetry.update();

//        Robot.startOdometryThread();

        Mecanum motionProfile = (Mecanum) Robot.motionProfile;
        timer.reset();

        while (opModeIsActive()) {
            timer.reset();
            joystickCancel = gamepad1.b || gamepad1.left_stick_x != 0 || gamepad1.right_stick_y != 0 || gamepad2.right_bumper ? 1 : 0;

            int shoot;
            if(gamepad1.a) {
                shoot = 1;
            } else if(gamepad1.b || gamepad1.y || gamepad1.x) {
                shoot = 2;
            } else {
                shoot = 0;
            }

            if(!autoRunning) {
                subsystems.drivetrain.mecanumDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, false, false, gamepad1.y, gamepad1.x, gamepad1.left_bumper, gamepad1.right_bumper);
                subsystems.intake.run(gamepad2.b, gamepad2.a, gamepad2.right_bumper);
                subsystems.shooter.run(gamepad2.left_bumper, gamepad2.right_bumper, shoot, gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.dpad_down);
                subsystems.wobbleGrabber.run(gamepad2.x, gamepad2.y, -gamepad2.left_stick_y * 0.5);
            }
//            motionProfile.update();
//            if(gamepad1.left_trigger > 0) {
//                Robot.setRobotPos(new Point(Robot.getRobotX(), 21.6));
//            }
//            if(gamepad1.right_trigger  > 0) {
//                Robot.setRobotPos(new Point(138.34, Robot.getRobotY()));
//            }

            if(gamepad1.a && !autoRunning) {
                autoRunning = true;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        Robot.setRobotPos(new Point(32.5, 169));
                        Robot.setRobotAngle(0.0);
                        ArrayList<ArrayList<PathPoint>> sections = new ArrayList<>();

                        ArrayList<PathPoint> powershotPos = new ArrayList<>();
                        powershotPos.add(new PathPoint(32.5, 169, -0.5, 1, 12, 0));
                        powershotPos.add(new PathPoint(32.5, 169, -0.5, 1, 12, 0));
                        powershotPos.add(new PathPoint(100, 170, -0.5, 1, 12, 0));
                        powershotPos.add(new PathPoint(100, 170, -0.5, 1, 12, 0));

                        sections.add(powershotPos);

                        final PurePursuit controller = new PurePursuit(sections);

                        new Thread(new Runnable() {
                            @Override
                            public void run() {
                                controller.run();
                            }
                        }).start();

//                        new Thread(new Runnable() {
//                            @Override
//                            public void run() {
//                                while(gamepad1.left_stick_x == 0) {}
//                                controller.stop = true;
//                                autoRunning = false;
//                            }
//                        }).start();

                        subsystems.drivetrain.resetAiming();
                        while(controller.getCurrentSection() == 0 && !isStopRequested() && joystickCancel == 0) {
                            subsystems.shooter.setPowers(0.895);
                            subsystems.shooter.setHopperPosition(0);
                            subsystems.shooter.setPowerShotPosition(0.04);

                        }
                        safeSleep(50, joystickCancel != 0);
                        subsystems.shooter.setPowers(0.895);
                        while(!subsystems.drivetrain.isAimed() && !isStopRequested() && joystickCancel == 0) {
                            subsystems.shooter.setHopperPosition(0.0);
                            subsystems.shooter.setPowers(0.895);
                            subsystems.drivetrain.aim(1, 0,0.005);
                        }
                        subsystems.drivetrain.run(0, 0);
                        safeSleep(100, joystickCancel != 0);
                        subsystems.shooter.setHopperPosition(0.18);
                        safeSleep(500, joystickCancel != 0);
                        subsystems.drivetrain.resetAiming();
                        while(!subsystems.drivetrain.isAimed() && !isStopRequested()&& joystickCancel == 0) {
                            subsystems.shooter.setHopperPosition(0.0);
                            subsystems.shooter.setPowers(0.895);
                            subsystems.drivetrain.aim(2, 0, 0.005);
                        }
                        subsystems.drivetrain.run(0, 0);
                        safeSleep(100, joystickCancel != 0);
                        subsystems.shooter.setHopperPosition(0.18);
                        safeSleep(500, joystickCancel != 0);
                        subsystems.drivetrain.resetAiming();
                        while(!subsystems.drivetrain.isAimed() && !isStopRequested() && joystickCancel == 0) {
                            subsystems.shooter.setHopperPosition(0.0);
                            subsystems.shooter.setPowers(0.895);
                            subsystems.drivetrain.aim(3, 0, 0.005);
                        }
                        subsystems.drivetrain.run(0, 0);
                        safeSleep(100, joystickCancel != 0);
                        subsystems.shooter.setHopperPosition(0.18);
                        safeSleep(500, joystickCancel != 0);
                        subsystems.drivetrain.resetAiming();
                        subsystems.shooter.setHopperPosition(0.0);
                        subsystems.shooter.setPowers(0.0);
                        subsystems.shooter.setPowerShotPosition(0.65);

                        controller.stop = true;
                        autoRunning = false;

                    }
                }).start();

            }

            motionProfile.update();


            //TODO disable debug for competition code
            if(debug) {
                debug();
            }

//            telemetry.addData("left", motionProfile.getLeft().getPosition());
//            telemetry.addData("right", motionProfile.getRight().getPosition());
//            telemetry.addData("horizontal", motionProfile.getHorizontal().getPosition());
//            telemetry.addData("loop time", time);
//            telemetry.update();
            double TARGET_UPS = 100.0;
            while(timer.milliseconds() < 1000.0 / TARGET_UPS) {
//               sleep(1);
            }
            time = timer.milliseconds();
//            timer.reset();
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
        telemetry.addData("robot angle", Robot.getRobotAngleDeg());
//        telemetry.addData("zAngle", subsystems.drivetrain.getZAngle());
    }

}
