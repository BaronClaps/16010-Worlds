package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

import java.util.Arrays;

@Autonomous
public class TestingAuto extends CommandOpMode {
    Alliance a = Alliance.RED;
    Paths p;
    Robot r;

    public void init() {
        r = new Robot(hardwareMap,  Alliance.RED);
        p = new Paths(r);

        r.follower.setStartingPose(Paths.start);
        r.shooter.setPower(0);
        r.turret.face(r.getShootTarget(), p.score);
        r.transfer.close();
        r.intake.raise();
    }

    public void start() {
        r.shooter.on();
        r.shooter.close();
        schedule(
                Commands.infinite(() -> {
                    r.periodic();

                    telemetry.addData("LoopTime Hz", r.getLoopTimeHz());
                    telemetry.addData("Pose", r.follower.getPose());
                    telemetry.addData("Target", r.getShootTarget());
                    telemetry.addData("Shooter Velocity", r.shooter.getVelocity());
                    telemetry.update();
                }),
                Groups.sequential(
                                p.preload(),
                                r.shoot(p.score),
                                r.intake(),
                                p.intakeSpike2()
                                        .raceWith(Commands.waitMs(5000.0)),
                                p.scoreSpike2(),
                                r.shoot(p.score),
                                r.intake(),
                                p.intakeGate()
                                        .raceWith(Commands.waitMs(4000.0)),
                                Commands.waitMs(1000.0),
                                p.scoreGate(),
                                r.shoot(p.score),
                                r.intake(),
                                p.intakeGate()
                                        .raceWith(Commands.waitMs(4000.0)),
                                Commands.waitMs(1000.0),
                                p.scoreGate(),
                                r.shoot(p.score),
                                r.intake(),
                                p.intakeGate()
                                        .raceWith(Commands.waitMs(4000.0)),
                                Commands.waitMs(1000.0),
                                p.scoreGate(),
                                r.shoot(p.score),
                                r.intake(),
                                p.intakeSpike3()
                                        .raceWith(Commands.waitMs(5000.0)),
                                p.scoreSpike3(),
                                r.shoot(p.score),
                                r.intake(),
                                p.intakeSpike1()
                                        .raceWith(Commands.waitMs(2000.0)),
                                p.scoreSpike1(),
                                r.shoot(p.score)
                        )
        );
    }

    public void stop() {
        super.stop();
        r.saveEnd();
    }
}
