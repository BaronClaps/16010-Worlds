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

        r.follower.setStartingPose(p.start);

        r.turret.resetTurret();
//        r.s.setHood(0.5);
//        r.s.setTarget(1800);

        r.setShootTarget();
        r.turret.automatic();
        r.turret.setPowerZero();
        r.transfer.openBottomGate();
        r.transfer.disengageKicker();
        r.transfer.disableAutoRotate();
        r.transfer.disableSort();
        r.transfer.moveTo(2);
        r.shooter.setPower(0);
    }

    public void start() {
        r.shooter.on();
        r.turret.on();
        r.transfer.closeTopGate();
     //   r.s.close();
        schedule(
                Commands.infinite(() -> {
                    r.periodic();
//                    double dist = r.getShootTarget().distanceFrom(r.f.getPose()) + 8;
//                    boolean close = r.f.getPose().getY() > 36;
//                    r.s.forDistance(dist, close);
//                    r.s.forPose(r.f.getPose(), r.getShootTarget(), close);
//                    r.s.setTarget(shootTarget);
//                    r.s.setHood(hoodTarget);
                    r.turret.face(r.getShootTarget(), r.follower.getPose());

                    telemetry.addData("LoopTime Hz", r.getLoopTimeHz());
                    telemetry.addData("Slots", Arrays.toString(r.transfer.slots));
                    telemetry.addData("Shooter Velocity", r.shooter.getVelocity());
                    telemetry.addData("Turret Error", r.turret.getError());
                    telemetry.addData("Pose", r.follower.getPose());
                    telemetry.addData("Target", r.getShootTarget());
                    telemetry.update();
                }),
                Groups.sequential(
                                p.preload(),
                                r.shootPassthrough(),
                                r.intakePassthrough(),
                                p.intakeSpike2()
                                        .raceWith(Commands.waitMs(5000.0)),
                                p.scoreSpike2(),
                                r.shootPassthrough(),
                                r.intakePassthrough(),
                                p.intakeGate()
                                        .raceWith(Commands.waitMs(4000.0)),
                                Commands.waitMs(1000.0),
                                p.scoreGate(),
                                r.shootPassthrough(),
                                r.intakePassthrough(),
                                p.intakeGate()
                                        .raceWith(Commands.waitMs(4000.0)),
                                Commands.waitMs(1000.0),
                                p.scoreGate(),
                                r.shootPassthrough(),
                                r.intakePassthrough(),
                                p.intakeGate()
                                        .raceWith(Commands.waitMs(4000.0)),
                                Commands.waitMs(1000.0),
                                p.scoreGate(),
                                r.shootPassthrough(),
                                r.intakePassthrough(),
                                p.intakeSpike3()
                                        .raceWith(Commands.waitMs(5000.0)),
                                p.scoreSpike3(),
                                r.shootPassthrough(),
                                r.intakePassthrough(),
                                p.intakeSpike1()
                                        .raceWith(Commands.waitMs(2000.0)),
                                p.scoreSpike1(),
                                r.shootPassthrough()

//                                r.shootSpindexUnsorted(),
//                                r.intakeSpindexUnsorted(),
//                                p.intakeSpike1()
//                                        .raceWith(Commands.waitMs(4000.0)),
//                                Groups.race(
//                                        p.hitGateAfterFirst()
//                                                .with(
//                                                        Commands.waitMs(500.0)
//                                                                .then(
//                                                                        r.i.off())),
//                                        Commands.waitMs(4000.0)
//                                ),
//                                // Commands.waitMs(),
//                                r.i.in(),
//                                p.scoreHitGate(),
//                                r.shootSpindexUnsorted(),
//                                r.intakeSpindexUnsorted(),
//                                p.intakeSpike2()
//                                        .raceWith(Commands.waitMs(4000.0)),
//
//                                p.scoreSpike2(),
//                                r.shootSpindexUnsorted(),
//                                r.intakeSpindexUnsorted(),
//                                p.intakeSpike3()
//                                        .raceWith(Commands.waitMs(4000.0)),
//                                Commands.waitMs(1000.0),
//                                p.scoreSpike3(),
//                                r.shootSpindexUnsorted(),
//                                r.intakeSpindexUnsorted(),
//                                p.intakeCorner()
//                                        .raceWith(Commands.waitMs(6000.0)),
//                                p.scoreCorner(),
//                                r.shootSpindexUnsorted(),
//                                p.park(),
//                                Commands.instant(r.s::off),
//                                r.i.off()
                        )
        );
    }

    public void stop() {
        super.stop();
        r.saveEnd();
    }
}
