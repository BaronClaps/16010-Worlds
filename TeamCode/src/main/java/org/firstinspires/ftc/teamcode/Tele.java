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
    public int shooting = 0;
    public double speed = 1, intakeOn = 0;
    public static double shootTarget = 1025, timeToStopIntake = .1, timeToOpenGate = .25, timeToShoot = 0.5, slowSpeed = .5;
    private final Timer shootTimer = new Timer();
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
        robot.intake.in();
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

        if (gamepad2.rightBumperWasPressed())
            if (intakeOn == 1)
                intakeOn = 0;
            else
                intakeOn = 1;

        if (gamepad2.leftBumperWasPressed())
            if (intakeOn == 2)
                intakeOn = 0;
            else
                intakeOn = 2;

        if (intakeOn == 1) {
            robot.intake.in();
            robot.transfer.in();
        } else if (intakeOn == 2) {
            robot.intake.out();
            robot.transfer.out();
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
        }

        if (gamepad1.leftBumperWasPressed())
            if (speed != 1)
                speed = 1;
            else
                speed = slowSpeed;

        if (shoot) {
            robot.shooter.on();
            if (manual) {
                robot.shooter.setTarget(shootTarget);

                if (gamepad2.dpadUpWasPressed())
                    shootTarget += 50;
                if (gamepad2.dpadDownWasPressed())
                    shootTarget -= 50;

                if (gamepad2.rightBumperWasPressed())
                    robot.turret.manual(0.05);
                if (gamepad2.leftBumperWasPressed())
                    robot.turret.manual(-0.05);

            } else {
                double dist = robot.getShootTarget().distanceFrom(robot.follower.getPose());
                boolean close = robot.follower.getPose().getY() > 48;
                robot.shooter.forDistance(dist, close);
               // robot.shooter.setTarget(shootTarget); // TODO: Regression
                robot.turret.face(robot.getShootTarget(), robot.follower.getPose());
            }
        } else {
            robot.shooter.off();
        }

        if (gamepad2.leftTriggerWasPressed())
            shoot = !shoot;

        if (gamepad2.rightTriggerWasPressed() && shoot) {
            shooting = 1;
            shootTimer.resetTimer();
            intakeOn = 0;
        }

        if (shooting == 1 && shootTimer.getElapsedTimeSeconds() > timeToStopIntake) {
            shooting = 2;
            shootTimer.resetTimer();
            robot.transfer.open();
        }

        if (shooting == 2 && shootTimer.getElapsedTimeSeconds() > timeToOpenGate) {
            shooting = 3;
            shootTimer.resetTimer();
            intakeOn = 1;
        }

        if (shooting == 3 && shootTimer.getElapsedTimeSeconds() > timeToShoot) {
            shooting = 0;
            intakeOn = 1;
            shootTimer.resetTimer();
            robot.transfer.close();
        }

        if (gamepad1.aWasPressed()) {
            if (robot.alliance.equals(Alliance.BLUE)) {
                robot.follower.setPose(new Pose(129.44,80.25, Math.toRadians(0)).mirror());
            } else {
                robot.follower.setPose(new Pose(129.44, 80.25, Math.toRadians(0)));
            }
        }

        if (gamepad2.bWasPressed())
            manual = !manual;

        if (gamepad1.yWasPressed())
            field = !field;

        multipleTelemetry.addData("Pose", robot.follower.getPose());
        multipleTelemetry.addData("Goal Target", robot.getShootTarget());
        multipleTelemetry.addLine();
        multipleTelemetry.addData("Shooter Velocity", robot.shooter.getVelocity());
        multipleTelemetry.addData("Shooter Target", robot.shooter.getTarget());
        multipleTelemetry.addLine();
        multipleTelemetry.addData("Manual", manual);
        multipleTelemetry.addData("Shoot Target for Manual", shootTarget);
        multipleTelemetry.addLine();
        multipleTelemetry.addData("Field Centric?", field);
        multipleTelemetry.update();

    }
}