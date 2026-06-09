package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

@Config
public class Close extends CommandOpMode {
    Alliance a;
    ClosePaths p;
    Robot robot;
    MultipleTelemetry telemetryM;
    boolean intakeTime, curr, prev, full;
    Timer intakeTimer = new Timer(), opModeTimer = new Timer();
    double intakeDist;
    public static double tValueToShoot = .4;

    public Close(Alliance a) {
        this.a = a;
    }

    public void init() {
        robot = new Robot(hardwareMap, a);
        p = new ClosePaths(robot);
        robot.follower.setStartingPose(ClosePaths.start);
        robot.follower.update();

        robot.shooter.setPower(0);
        robot.turret.setYaw(0);
        robot.transfer.close();
        robot.intake.raise();

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryM.addData("Pose", robot.follower.getPose());
        telemetryM.update();

        schedule(
                Commands.infinite(() -> {
                    robot.periodic();

                    if (robot.loops % 3 == 0) {
                        intakeDist = robot.intake.getDistance();
                        curr = robot.intake.isDetected(intakeDist);
                    }

                    if (curr && !prev) {
                        intakeTimer.resetTimer();
                    }

                    if (!intakeTime)
                        intakeTime = intakeTimer.getElapsedTimeSeconds() >= .5;


                    if (curr && intakeTime) {
                        if (!robot.shooter.atTarget())
                            robot.intake.light.orange();
                        else
                            robot.intake.light.green();

                        full = true;
                    } else {
                        if (!robot.shooter.atTarget())
                            robot.intake.light.violet();
                        else
                            robot.intake.light.blue();
                        full = false;
                    }

                    prev = curr;

                    telemetryM.addData("LoopTime Hz", robot.getLoopTimeHz());
                    telemetryM.addData("Pose", robot.follower.getPose());
                    telemetryM.addData("T Value", robot.follower.getCurrentTValue());
                    telemetryM.addData("Target", robot.getShootTarget());
                    telemetryM.addData("Shooter Velocity", robot.shooter.getVelocity());
                    telemetryM.update();
                }),
                Groups.sequential(
                        p.preload()
                                .with(
                                        waitUntil(() -> robot.follower.getCurrentTValue() > .85)
                                                .then(robot.shootNoSOTM(p.score))
                                ),
                        robot.intakeLowered(),
                        p.intakeSpike2()
                                .raceWith(waitMs(3000.0)),
                        p.scoreSpike2()
                                .with(
                                        waitMs(250.0)
                                                .then(
                                                        robot.intake.raiseCommand()
                                                ),
                                        waitUntil(() -> robot.follower.getCurrentTValue() > .3)
                                                .then(robot.shoot(p.score))
                                ),
                        robot.intakeLowered(),
                        p.intakeGate()
                                .raceWith(waitMs(2000.0)),
                        waitMs(1500.0)
                                .raceWith(Commands.waitMs(500.0).then(Commands.waitUntil(() -> full))),
                        p.scoreGate()
                                .with(
                                        waitMs(250.0)
                                                .then(
                                                        robot.intake.raiseCommand()
                                                ),
                                        waitUntil(() -> robot.follower.getCurrentTValue() > tValueToShoot)
                                                .then(robot.shoot(p.score))
                                ),
                        robot.intakeLowered(),
                        p.intakeGate()
                                .raceWith(waitMs(2000.0)),
                        waitMs(1500.0)
                                .raceWith(Commands.waitMs(500.0).then(Commands.waitUntil(() -> full))),
                        p.scoreGate()
                                .with(
                                        waitMs(250.0)
                                                .then(
                                                        robot.intake.raiseCommand()
                                                ),
                                        waitUntil(() -> robot.follower.getCurrentTValue() > tValueToShoot)
                                                .then(robot.shoot(p.score))
                                ),
                        robot.intakeLowered(),
                        p.intakeSpike1()
                                .raceWith(waitMs(2000.0)),
                        p.scoreSpike1()
                                .with(
                                        waitMs(250.0)
                                                .then(
                                                        robot.intake.raiseCommand()
                                                ),
                                        waitUntil(() -> robot.follower.getCurrentTValue() > .4)
                                                .then(robot.shoot(p.score))
                                ),
                        robot.intakeLowered(),
                        p.intakeGate()
                                .raceWith(waitMs(2000.0)),
                        waitMs(1500.0)
                                .raceWith(Commands.waitMs(500.0).then(Commands.waitUntil(() -> full))),
                        p.scoreGate()
                                .with(
                                        waitMs(250.0)
                                                .then(
                                                        robot.intake.raiseCommand()
                                                ),
                                        waitUntil(() -> robot.follower.getCurrentTValue() > tValueToShoot)
                                                .then(robot.shoot(p.score))
                                ),
                        robot.intakeLowered(),
                        p.intakeGate()
                                .raceWith(waitMs(2000.0)),
                        waitMs(1500.0)
                                .raceWith(Commands.waitMs(500.0).then(Commands.waitUntil(() -> full))),
                        p.scoreGate()
                                .with(
                                        waitMs(250.0)
                                                .then(
//                                                        robot.intake.raiseCommand()
                                                ),
                                        waitUntil(() -> robot.follower.getCurrentTValue() > tValueToShoot)
                                                .then(robot.shoot(p.score))
                                ),
                        conditional(
                                () -> opModeTimer.getElapsedTimeSeconds() < 27.5,
                                sequential(
                                        robot.intakeLowered(),
                                        p.intakeGate()
                                                .raceWith(waitMs(2000.0)),
                                        waitMs(1500.0)
                                                .raceWith(Commands.waitMs(500.0).then(Commands.waitUntil(() -> full))),
                                        p.scoreGate()
                                                .with(
                                                        waitMs(250.0)
                                                                .then(
//                                                        robot.intake.raiseCommand()
                                                                ),
                                                        waitUntil(() -> robot.follower.getCurrentTValue() > tValueToShoot)
                                                                .then(robot.shoot(p.score))
                                                )
                                ),
                                p.park()
                        )

                )
        );
    }

    public void start() {
        robot.shooter.on();
        robot.shooter.close();
        robot.intake.off();
        robot.transfer.open();
        robot.turret.face(robot.getShootTarget(), p.score);
        opModeTimer.resetTimer();
    }

    public void stop() {
        robot.saveEnd();
        robot.transfer.open();
        super.stop();
    }
}
