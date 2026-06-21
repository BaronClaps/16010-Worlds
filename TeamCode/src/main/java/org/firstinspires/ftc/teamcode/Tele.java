package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.SOTM;

import static org.firstinspires.ftc.teamcode.Robot.defaultPose;

@Config
public class Tele extends OpMode {
    Alliance alliance;
    Robot robot;

    public boolean shoot = false;
    public boolean manual = false;
    public boolean field = false;
    public boolean raised = true;
    public boolean close = true;
    public boolean prev = false, curr = false, intakeTime = false, closeMode = true, twoDown = false, openingGate = false, bypassOpenWait = false;
    public int shooting = 0;
    public double speed = 1, intakeOn = 1, intakePower = 1, dist, intakeDist;
    public static double shootTarget = 1100, timeToShootClose = 0.5, timeToShootFar = .75, transferPower = 0.5, timeFor3rd = .15, transferIntakingPower = 0.5; // .5;
    private final Timer shootTimer = new Timer(), intakeTimer = new Timer(), openGateTimer = new Timer();
    MultipleTelemetry multipleTelemetry;

    public Tele(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, alliance);
        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    public void start() {
        robot.follower.setStartingPose(defaultPose);
        robot.setShootTarget();
        robot.follower.startTeleopDrive();
        robot.turret.set(0.5);
        robot.shooter.off();
        robot.transfer.close();
        robot.intake.raise();
        shootTimer.resetTimer();
    }

    @Override
    public void loop() {
        robot.periodic();

        if (field)
            robot.follower.setTeleOpDrive(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x, speed * -gamepad1.right_stick_x * .75, false, robot.alliance == Alliance.BLUE ? Math.toRadians(180) : 0);
        else
            robot.follower.setTeleOpDrive(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x, speed * -gamepad1.right_stick_x * .75, true);

        if (gamepad1.rightBumperWasPressed() || gamepad1.rightBumperWasPressed())
            if (intakeOn == 1)
                intakeOn = 0;
            else
                intakeOn = 1;

        if (gamepad2.leftBumperWasPressed())
            if (intakeOn == 2)
                intakeOn = 0;
            else
                intakeOn = 2;

        if (gamepad1.xWasPressed()) {
            twoDown = !twoDown;

            if (twoDown)
                robot.intake.two();
            else
                robot.intake.lower();
        }

        if (gamepad2.yWasPressed()) {
            openGateTimer.resetTimer();
            openingGate = true;
            intakeOn = 0;
        }

        if (gamepad1.dpadUpWasPressed())
            closeMode = true;

        if (gamepad1.dpadDownWasPressed())
            closeMode = false;

        if (intakeOn == 1) {
            robot.intake.set(intakePower);
            robot.transfer.set(transferPower);

            /*
            if (shooting > 0) {
                robot.intake.set(transferPower);
                robot.transfer.set(transferPower);
            } else {
                robot.intake.set(transferPower);
                robot.transfer.set(.3);
            }
             */
        } else if (intakeOn == 2) {
            robot.intake.out();
            robot.transfer.out();

            if (curr)
                curr = false;
        } else {
            robot.intake.off();
            robot.transfer.off();
        }

        if (gamepad1.rightBumperWasPressed()) {
            if (raised)
                robot.intake.lower();
            else
                robot.intake.raise();
            raised = !raised;
            twoDown = false;
        }

        if (shoot) {
            robot.shooter.on();
            if (manual) {
                robot.shooter.setTarget(shootTarget);

                if (gamepad2.dpadUpWasPressed())
                    shootTarget += 20;
                if (gamepad2.dpadDownWasPressed())
                    shootTarget -= 20;

                if (gamepad2.dpadRightWasPressed())
                    robot.turret.manual(0.05);
                if (gamepad2.dpadLeftWasPressed())
                    robot.turret.manual(-0.05);

            } else {
                dist = robot.getShootTarget().distanceFrom(robot.follower.getPose());
                close = robot.follower.getPose().getY() > 48;

//                Pose virtualRobot = SOTM.calculateVirtualRobot(robot.follower.getPose(), robot.follower.getVelocity(), dist);
//                double virtualDist = robot.getShootTarget().distanceFrom(virtualRobot);
//
//                if (!closeMode)
//                    robot.shooter.forFar(virtualDist);
//                else
//                    robot.shooter.forClose(virtualDist);
//
//                robot.turret.face(robot.getAimTarget(), virtualRobot);

                if (!closeMode)
                    robot.shooter.forFar(dist);
                else
                    robot.shooter.forClose(dist);

                robot.turret.face(robot.getAimTarget(), robot.follower.getPose());
            }
        } else {
            robot.shooter.off();
        }

        if (gamepad1.leftTriggerWasPressed())
            shoot = !shoot;

        if (gamepad1.rightTriggerWasPressed() && shoot && shooting == 0) {
            shooting = 1;
            shootTimer.resetTimer();
            intakeOn = 0;
        }

        if (shooting == 1) {
            shooting = 2;
            robot.transfer.open();
            shootTimer.resetTimer();
        }

        if (shooting == 2 && shootTimer.getElapsedTimeSeconds() >= .2) {
            shooting = 3;

            if (!closeMode) {
                transferPower = 0.7;
                intakePower = 0.7;
            } else {
                transferPower = 1;
                intakePower = 1;
            }

            intakeOn = 1;
            shootTimer.resetTimer();
        }

        if (shooting == 3 && ((closeMode && shootTimer.getElapsedTimeSeconds() > timeToShootClose || (!closeMode && shootTimer.getElapsedTimeSeconds() > timeToShootFar))))  {
            shooting = 0;
            intakeOn = 1;
            transferPower = transferIntakingPower;
            intakePower = 1;
            shootTimer.resetTimer();
            robot.transfer.close();
            curr = false;
        }

        if (gamepad1.aWasPressed()) {
            if (robot.alliance.equals(Alliance.BLUE)) {
                robot.follower.setPose(new Pose(128.062, 78.125, Math.toRadians(90)).mirror());
            } else {
                robot.follower.setPose(new Pose(128.062, 78.125, Math.toRadians(90)));
            }
        }

        if (gamepad1.bWasPressed()) {
            if (robot.alliance.equals(Alliance.BLUE)) {
                robot.follower.setPose(new Pose(134.5, 11.13, Math.toRadians(180)));
            } else {
                robot.follower.setPose(new Pose(134.5, 11.13, Math.toRadians(180)).mirror());
            }
        }

        if (gamepad2.bWasPressed())
            manual = !manual;

        if (gamepad1.yWasPressed())
            field = !field;

        if (robot.loops % 3 == 0) {
            intakeDist = robot.intake.getDistance();
            curr = robot.intake.isDetected(intakeDist);
        }

        if (curr != prev) {
            intakeTimer.resetTimer();
        }

        if (!intakeTime)
            intakeTime = intakeTimer.getElapsedTimeSeconds() >= timeFor3rd;

        if (curr && intakeTime) {
            if (!robot.shooter.atTarget())
                robot.intake.light.orange();
            else
                robot.intake.light.green();
        } else {
            if (!robot.shooter.atTarget())
                robot.intake.light.violet();
            else
                robot.intake.light.blue();
        }

        prev = curr;

        double sAmps = robot.shooter.getCurrent();
        double tAmps = robot.transfer.getCurrent();
        double iAmps = robot.intake.getCurrent();

        multipleTelemetry.addData("Pose", robot.follower.getPose());
        multipleTelemetry.addData("Goal Target", robot.getShootTarget());
        multipleTelemetry.addData("Distance", dist);
        multipleTelemetry.addData("Close?", close);
        multipleTelemetry.addLine();
        multipleTelemetry.addData("Intake Amps", iAmps);
        multipleTelemetry.addData("Transfer Amps", tAmps);
        multipleTelemetry.addData("Shooter Amps", sAmps);
        multipleTelemetry.addData("Total Amps", sAmps + tAmps + iAmps);
        multipleTelemetry.addLine();
        multipleTelemetry.addData("Turret Angle", robot.turret.getYaw());
        multipleTelemetry.addData("Turret Position", robot.turret.get());
        multipleTelemetry.addLine();
        multipleTelemetry.addData("Shooter Velocity", robot.shooter.getVelocity());
        multipleTelemetry.addData("Shooter Target", robot.shooter.getTarget());
        multipleTelemetry.addLine();
        multipleTelemetry.addData("Looptime Hz", robot.getLoopTimeHz());
        multipleTelemetry.addData("Intake Distance", intakeDist);
        multipleTelemetry.addData("Intake 3rd Detected", curr);
        multipleTelemetry.addLine();
        multipleTelemetry.addData("Manual", manual);
        multipleTelemetry.addData("Shoot Target for Manual", shootTarget);
        multipleTelemetry.addLine();
        multipleTelemetry.addData("Field Centric?", field);
        multipleTelemetry.update();

    }
}