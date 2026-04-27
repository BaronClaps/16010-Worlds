package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

@Config
public class Far extends CommandOpMode {
    Alliance a;
    FarPaths p;
    Robot robot;
    MultipleTelemetry telemetryM;
    public static double tValueToRaise = .1;

    public Far(Alliance a) {
        this.a = a;
    }

    public void init() {
        robot = new Robot(hardwareMap, a);
        p = new FarPaths(robot);

        robot.follower.setStartingPose(FarPaths.start);
        robot.shooter.setPower(0);
        robot.turret.setYaw(0);
        robot.transfer.close();
        robot.intake.raise();

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
                        robot.shoot(p.score),
                        robot.intakeLowered(),
                        p.intakeSpike3()
                                .raceWith(waitMs(5000.0)),
                        p.scoreSpike3()
                                .with(
                                        waitMs(150.0)
                                                .then(robot.intake.raiseCommand())
                                ),
                        robot.shoot(p.score),
                        robot.intakeLowered(),
                        p.intakeCorner()
                                .raceWith(
                                    waitMs(2000.0)
                                ),
                        p.scoreCorner()
                                .with(
                                        waitMs(150.0)
                                                .then(robot.intake.raiseCommand())
                                ),
                        robot.shoot(p.score),
                        robot.intakeLowered(),
                        p.intakeCorner()
                                .raceWith(
                                        waitMs(2000.0)
                                ),
                        p.scoreCorner()
                                .with(
                                        waitMs(150.0)
                                                .then(robot.intake.raiseCommand())
                                ),
                        robot.shoot(p.score),
                        robot.intakeLowered(),
                        p.intakeCorner()
                                .raceWith(
                                        waitMs(2000.0)
                                ),
                        p.scoreCorner()
                                .with(
                                        waitMs(150.0)
                                                .then(robot.intake.raiseCommand())
                                ),
                        robot.shoot(p.score),
                        robot.intakeLowered(),
                        p.intakeCorner()
                                .raceWith(
                                        waitMs(2000.0)
                                ),
                        p.scoreCorner()
                                .with(
                                        waitMs(150.0)
                                                .then(robot.intake.raiseCommand())
                                ),
                        robot.shoot(p.score),
                        robot.intakeLowered(),
                        p.intakeGate()
                                .raceWith(
                                        waitMs(2000.0)
                                ),
                        p.scoreGate()
                                .with(
                                        waitMs(150.0)
                                                .then(robot.intake.raiseCommand())
                                ),
                        robot.shoot(p.score),
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
