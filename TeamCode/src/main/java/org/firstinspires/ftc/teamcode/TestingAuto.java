package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

@Autonomous
@Config
public class TestingAuto extends CommandOpMode {
    Alliance a = Alliance.RED;
    Paths p;
    Robot robot;
    MultipleTelemetry telemetryM;
    public static double tValueToShoot = .5;

    public void init() {
        robot = new Robot(hardwareMap, a);
        p = new Paths(robot);

        robot.follower.setStartingPose(Paths.start);
        robot.shooter.setPower(0);
        robot.turret.setYaw(0);
        robot.transfer.close();
        robot.intake.raise();

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void start() {
        robot.shooter.on();
        robot.shooter.close();
        robot.turret.face(robot.getShootTarget(), p.score);
        schedule(
                Commands.infinite(() -> {
                    robot.periodic();

                    telemetryM.addData("LoopTime Hz", robot.getLoopTimeHz());
                    telemetryM.addData("Pose", robot.follower.getPose());
                    telemetryM.addData("Target", robot.getShootTarget());
                    telemetryM.addData("Shooter Velocity", robot.shooter.getVelocity());
                    telemetryM.update();
                }),
                Groups.sequential(
                        p.preload(),
                        robot.shoot(p.score),
                        robot.intakeLowered(),
                        p.intakeSpike2()
                                .raceWith(waitMs(5000.0)),
                        p.scoreSpike2()
                                .with(
                                        waitMs(20.0)
                                                .then(
                                                        robot.intake.raiseCommand()
                                                )
                                )
                                .with(
                                        waitUntil(() -> robot.follower.getCurrentTValue() > tValueToShoot)
                                                .then(robot.shoot(p.score))
                                ),
                        robot.intakeRaised(),
                        p.intakeGate()
                                .raceWith(waitMs(4000.0)),
                        waitMs(1000.0),
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
                                .raceWith(waitMs(4000.0)),
                        waitMs(1500.0),
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
                                .raceWith(waitMs(5000.0)),
                        p.scoreSpike1()
                                .with(
                                        waitUntil(() -> robot.follower.getCurrentTValue() > .85)
                                                .then(robot.shoot(p.score))
                                ),
                        robot.intakeRaised(),
                        p.intakeGate()
                                .raceWith(waitMs(4000.0)),
                        waitMs(1500.0),
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
                                .raceWith(waitMs(4000.0)),
                        waitMs(1500.0),
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
//                        robot.intakeRaised(),
//                        p.intakeSpike3()
//                                .raceWith(waitMs(2000.0)),
//                        p.scoreSpike3(),
//                        robot.shoot(p.score)
                )
        );
    }

    public void stop() {
        super.stop();
        robot.saveEnd();
    }
}
