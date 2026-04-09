package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.waitMs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

@Autonomous
public class TestingAuto extends CommandOpMode {
    Alliance a = Alliance.BLUE;
    Paths p;
    Robot robot;
    MultipleTelemetry telemetryM;

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
                        robot.intakeSpike(),
                        p.intakeSpike2()
                                .raceWith(waitMs(5000.0)),
                        p.scoreSpike2()
                                .with(
                                        waitMs(250.0)
                                                .then(
                                                        robot.intake.raiseCommand()
                                                )
                                ),
                        robot.shoot(p.score),
                        robot.intake(),
                        p.intakeGate()
                                .raceWith(waitMs(4000.0)),
                        waitMs(1000.0),
                        p.scoreGate()
                                .with(
                                        waitMs(0.0)
                                                .then(
                                                        robot.intake.raiseCommand()
                                                )
                                ),
                        robot.shoot(p.score),
                        robot.intake(),
                        p.intakeGate()
                                .raceWith(waitMs(4000.0)),
                        waitMs(1000.0),
                        p.scoreGate()
                                .with(
                                        waitMs(0.0)
                                                .then(
                                                        robot.intake.raiseCommand()
                                                )
                                ),
                        robot.shoot(p.score),
                        robot.intake(),
                        p.intakeGate()
                                .raceWith(waitMs(4000.0)),
                        waitMs(1000.0),
                        p.scoreGate()
                                .with(
                                        waitMs(0.0)
                                                .then(
                                                        robot.intake.raiseCommand()
                                                )
                                ),
                        robot.shoot(p.score),
                        robot.intakeSpike(),
                        p.intakeSpike1()
                                .raceWith(waitMs(5000.0)),
                        p.scoreSpike1(),
                        robot.shoot(p.score),
                        robot.intakeSpike(),
                        p.intakeSpike3()
                                .raceWith(waitMs(2000.0)),
                        p.scoreSpike3(),
                        robot.shoot(p.score)
                )
        );
    }

    public void stop() {
        super.stop();
        robot.saveEnd();
    }
}
