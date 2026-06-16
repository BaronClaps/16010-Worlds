package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.waitMs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

@Config
public class Far extends CommandOpMode {
    Alliance a;
    FarPaths p;
    Robot robot;
    MultipleTelemetry telemetryM;
    boolean intakeTime, curr, prev, full;
    Timer intakeTimer = new Timer(), opModeTimer = new Timer();
    double intakeDist;

    public Far(Alliance a) {
        this.a = a;
    }

    public void init() {
        robot = new Robot(hardwareMap, a);
        p = new FarPaths(robot);

        robot.follower.setStartingPose(p.start);
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
                        intakeTime = intakeTimer.getElapsedTimeSeconds() >= 1;


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

                    robot.shooter.forDistance(robot.getShootTarget().distanceFrom(p.score), p.score.getY() > 48);
                    robot.turret.face(robot.getShootTarget(), p.score);

                    telemetryM.addData("LoopTime Hz", robot.getLoopTimeHz());
                    telemetryM.addData("Pose", robot.follower.getPose());
                    telemetryM.addData("T Value", robot.follower.getCurrentTValue());
                    telemetryM.addData("Target", robot.getShootTarget());
                    telemetryM.addData("Shooter Velocity", robot.shooter.getVelocity());
                    telemetryM.addData("turret angle", robot.turret.getYaw());
                    telemetryM.addData("full", full);
                    telemetryM.update();
                }),
                Groups.sequential(
                        p.preload()
                                .then(
                                        waitMs(250.0),
                                        robot.shootFar(p.score)
                                ),
                        spike(),
                        corner(),
                        between(),
                        spike(),
                        corner(),
                        between(),
                        corner(),
                        p.park()
                )
        );
    }

    public void start() {
        robot.shooter.on();
        robot.shooter.forFar(robot.getAimTarget().distanceFrom(p.score));
    }

    public void stop() {
        robot.transfer.open();
        robot.saveEnd();
        super.stop();
    }

    public CommandBuilder resetTimer() {
        return Commands.instant(() -> intakeTimer.resetTimer());
    }

    private CommandBuilder corner() {
        return Groups.sequential(
                robot.intake(),
                resetTimer(),
                p.intakeCorner()
                        .raceWith(
                                waitMs(3000.0),
                                Commands.waitMs(1000.0)
                                        .then(Commands.waitUntil(() -> full))
                        ),
                p.scoreCorner()
                        .with(
                                waitMs(1250.0)
                                        .then(
                                                robot.intake.offCommand()
                                        )
                        ),
                robot.shootFar(p.score)
        );
    }

    private CommandBuilder spike() {
        return Groups.sequential(
                robot.intake(),
                resetTimer(),
                p.intakeSpike3()
                        .raceWith(
                                waitMs(3000.0),
                                Commands.waitMs(1000.0)
                                        .then(Commands.waitUntil(() -> full))
                        ),
                p.scoreSpike3()
                        .with(
                                waitMs(1250.0)
                                        .then(
                                                robot.intake.offCommand()
                                        )
                        ),
                robot.shootFar(p.score)
        );
    }

    private CommandBuilder between() {
        return Groups.sequential(
                robot.intake(),
                resetTimer(),
                p.intakeBetween()
                        .raceWith(
                                waitMs(3000.0),
                                Commands.waitMs(1000.0)
                                        .then(Commands.waitUntil(() -> full))
                        ),
                p.scoreBetween()
                        .with(
                                waitMs(1250.0)
                                        .then(
                                                robot.intake.offCommand()
                                        )
                        ),
                robot.shootFar(p.score)
        );
    }
}
