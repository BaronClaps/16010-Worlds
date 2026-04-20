package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;

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
    Timer intakeTimer = new Timer();
    double intakeDist;
    public static double tValueToShoot = .5;

    public Close(Alliance a) {
        this.a = a;
    }

    public void init() {
        robot = new Robot(hardwareMap, a);
        p = new ClosePaths(robot);

        robot.follower.setStartingPose(ClosePaths.start);
        robot.shooter.setPower(0);
        robot.turret.setYaw(0);
        robot.transfer.close();
        robot.intake.raise();

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
                        robot.intake.light.green();
                        //full = true;
                    } else {
                        robot.intake.light.blue();
                        full = false;
                    }

                    prev = curr;

                    telemetryM.addData("LoopTime Hz", robot.getLoopTimeHz());
                    telemetryM.addData("Pose", robot.follower.getPose());
                    telemetryM.addData("Target", robot.getShootTarget());
                    telemetryM.addData("Shooter Velocity", robot.shooter.getVelocity());
                    telemetryM.update();
                }),
                Groups.sequential(
                        p.preload()
                                .with(
                                        waitUntil(() -> robot.follower.getCurrentTValue() > .75)
                                                .then(robot.shoot(p.score))
                                ),
                        robot.intakeLowered(),
                        p.intakeSpike2()
                                .raceWith(waitMs(3000.0)),
                        p.scoreSpike2()
                                .with(
                                        waitMs(20.0)
                                                .then(
                                                        robot.intake.raiseCommand()
                                                )
                                )
                                .with(
                                        waitUntil(() -> robot.follower.getCurrentTValue() > .85)
                                                .then(robot.shoot(p.score))
                                ),
                        robot.intakeRaised(),
                        p.intakeGate()
                                .raceWith(waitMs(2000.0)),
                        waitMs(1500.0)
                                .raceWith(Commands.waitUntil(() -> full)),
                        p.scoreGate()
                                .with(
                                        parallel(
                                                waitMs(0.0)
                                                        .then(
                                                                robot.intake.lowerCommand()
                                                        ),
                                                waitUntil(() -> robot.follower.getCurrentTValue() > tValueToShoot)
                                                        .then(robot.shoot(p.score))
                                        )
                                ),
                        robot.intakeRaised(),
                        p.intakeGate()
                                .raceWith(waitMs(2000.0)),
                        waitMs(1500.0)
                                .raceWith(Commands.waitUntil(() -> full)),
                        p.scoreGate()
                                .with(
                                        parallel(
                                                waitMs(0.0)
                                                        .then(
                                                                robot.intake.lowerCommand()
                                                        ),
                                                waitUntil(() -> robot.follower.getCurrentTValue() > tValueToShoot)
                                                        .then(robot.shoot(p.score))
                                        )
                                ),
                        robot.intakeLowered(),
                        p.intakeSpike1()
                                .raceWith(waitMs(2000.0)),
                        p.scoreSpike1()
                                .with(
                                        waitUntil(() -> robot.follower.getCurrentTValue() > .85)
                                                .then(robot.shoot(p.score))
                                ),
                        robot.intakeRaised(),
                        p.intakeGate()
                                .raceWith(waitMs(2000.0)),
                        waitMs(1500.0)
                                .raceWith(Commands.waitUntil(() -> full)),
                        p.scoreGate()
                                .with(
                                        parallel(
                                                waitMs(0.0)
                                                        .then(
                                                                robot.intake.lowerCommand()
                                                        ),
                                                waitUntil(() -> robot.follower.getCurrentTValue() > tValueToShoot)
                                                        .then(robot.shoot(p.score))
                                        )
                                ),
                        robot.intakeRaised(),
                        p.intakeGate()
                                .raceWith(waitMs(2000.0)),
                        waitMs(1500.0)
                                .raceWith(Commands.waitUntil(() -> full)),
                        p.scoreGate()
                                .with(
                                        parallel(
                                                waitMs(0.0)
                                                        .then(
                                                                robot.intake.lowerCommand()
                                                        ),
                                                waitUntil(() -> robot.follower.getCurrentTValue() > tValueToShoot)
                                                        .then(robot.shoot(p.score))
                                        )
                                ),
                        p.park()
                )
        );
    }

    public void start() {
        robot.shooter.on();
        robot.shooter.close();
        robot.turret.face(robot.getShootTarget(), p.score);
    }

    public void stop() {
        robot.saveEnd();
        super.stop();
    }
}
