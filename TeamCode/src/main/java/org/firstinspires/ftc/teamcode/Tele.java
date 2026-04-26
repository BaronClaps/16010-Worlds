package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.Alliance;

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
    public double speed = 1, intakeOn = 1, dist, intakeDist;
    public static double shootTarget = 1100, timeToStopIntake = .1, timeToOpenGate = .25, timeToShoot = 0.5, slowSpeed = .5, transferPower = 1, timeFor3rd = .15; // .5;
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
            robot.follower.setTeleOpDrive(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x, speed * -gamepad1.right_stick_x, false, robot.alliance == Alliance.BLUE ? Math.toRadians(180) : 0);
        else
            robot.follower.setTeleOpDrive(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x, speed * -gamepad1.right_stick_x, true);

        if (gamepad2.rightBumperWasPressed() /*&& gamepad1.leftBumperWasPressed()*/) //TODO: this is solo
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

        if (openGateTimer.getElapsedTimeSeconds() > timeToStopIntake && openingGate) {
            robot.transfer.open();
            openingGate = false;
        }

        if (gamepad1.rightTriggerWasPressed())
            closeMode = true;

        if (gamepad1.leftTriggerWasPressed())
            closeMode = false;

        if (intakeOn == 1) {
            robot.intake.set(transferPower);
            robot.transfer.set(transferPower);
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

                if (!close && !closeMode)
                    robot.shooter.forFar(dist);
                else if (close && !closeMode)
                    robot.shooter.setTarget(1000);
                else if (close)
                    robot.shooter.forClose(dist);
                else
                    robot.shooter.setTarget(700);

                robot.turret.face(robot.getAimTarget(), robot.follower.getPose());
            }
        } else {
            robot.shooter.off();
        }

        if (gamepad2.leftTriggerWasPressed())
            shoot = !shoot;

        if (gamepad2.rightTriggerWasPressed() && shoot) {
            if (robot.transfer.closed()) {
                shooting = 1;
                shootTimer.resetTimer();
                intakeOn = 0;
            } else {
                shooting = 2;
                bypassOpenWait = true;
            }
        }

        if (shooting == 1 && shootTimer.getElapsedTimeSeconds() > timeToStopIntake) {
            shooting = 2;
            shootTimer.resetTimer();
            robot.transfer.open();
        }

        if (shooting == 2 && (shootTimer.getElapsedTimeSeconds() > timeToOpenGate || bypassOpenWait)) {
            shooting = 3;
            shootTimer.resetTimer();
            bypassOpenWait = false;

            if (!closeMode)
                transferPower = .7;
            else
                transferPower = 1;

            intakeOn = 1;
        }

        if (shooting == 3 && shootTimer.getElapsedTimeSeconds() > timeToShoot) {
            shooting = 0;
            intakeOn = 1;
            transferPower = 1;
            shootTimer.resetTimer();
            robot.transfer.close();
            curr = false;
//            transferPower = .5;
        }

        if (gamepad1.aWasPressed()) {
            if (robot.alliance.equals(Alliance.BLUE)) {
                robot.follower.setPose(new Pose(129.44, 80.25, Math.toRadians(0)).mirror());
            } else {
                robot.follower.setPose(new Pose(129.44, 80.25, Math.toRadians(0)));
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

        multipleTelemetry.addData("Pose", robot.follower.getPose());
        multipleTelemetry.addData("Goal Target", robot.getShootTarget());
        multipleTelemetry.addData("Distance", dist);
        multipleTelemetry.addData("Close?", close);
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