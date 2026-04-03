package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

public class Auto extends CommandOpMode {
    Alliance a;
    Paths p;

    Robot r;

    public Auto(Alliance a) {
        this.a = a;
    }

    public void init() {
        r = new Robot(hardwareMap, this.a);
        p = new Paths(r);

        r.follower.setStartingPose(p.start);
        r.setShootTarget();
        r.shooter.setPower(0);
    }

        public void start() {
        r.shooter.on();
        r.shooter.close();
        schedule(
               Commands.infinite(() -> {
                   r.periodic();
                   r.turret.face(r.getShootTarget(), r.follower.getPose());

                   telemetry.addData("LoopTime Hz", r.getLoopTimeHz());
                   telemetry.addData("Shooter Velocity", r.shooter.getVelocity());
                   telemetry.addData("Pose", r.follower.getPose());
                   telemetry.addData("Target", r.getShootTarget());
                   telemetry.update();
               }),
                Groups.sequential(
                        p.preload(),
                        r.shootSpindexUnsorted(),
                        r.intakeSpindexUnsorted(),
                        p.intakeSpike1()
                                .raceWith(Commands.waitMs(3000.0)),
                        Groups.race(
                        p.hitGateAfterFirst()
                                        .with(
                                                Commands.waitMs(500.0)
                                                        .then(
                                                                r.intake.offCommand())),
                                Commands.waitMs(3000.0)
                        ),
                       // Commands.waitMs(),
                        r.intake.inCommand(),
                        p.scoreHitGate(),
                        r.shootSpindexUnsorted(),
                        r.intakeSpindexUnsorted(),
                        p.intakeSpike2()
                                .raceWith(Commands.waitMs(3000.0)),

                        p.scoreSpike2(),
                        r.shootSpindexUnsorted(),
                        r.intakeSpindexUnsorted(),
                        p.intakeSpike3()
                                .raceWith(Commands.waitMs(3000.0)),
                        Commands.waitMs(1000.0),
                        p.scoreSpike3(),
                        r.shootSpindexUnsorted(),
                        r.intakeSpindexUnsorted(),
                        p.intakeCorner()
                                .raceWith(Commands.waitMs(6000.0)),
                        p.scoreCorner(),
                        r.shootSpindexUnsorted(),
                        p.park(),
                        Commands.instant(r.shooter::off),
                        r.intake.offCommand()
                )
                        .with(
                                Commands.waitMs(29250.0)
                                        .then(
                                                Commands.instant(r.follower::breakFollowing),
                                                Commands.instant(() -> r.follower.holdPoint(r.follower.getPose(), false))
                                        )
                        )
        );
    }

    public void stop() {
        super.stop();
        r.saveEnd();

    }
}
