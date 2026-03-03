package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

import java.util.Arrays;

@Autonomous
public class TestingAuto extends CommandOpMode {
    Alliance a = Alliance.BLUE;
    Paths p;

    Robot r;

    public void init() {
        r = new Robot(hardwareMap, this.a);
        p = new Paths(r);

        r.f.setStartingPose(p.start);

        r.t.resetTurret();
        r.s.setHood(0.5);
        r.s.setTarget(1800);

        r.setShootTarget();
        r.t.setPowerZero();
        r.p.closeTopGate();
        r.p.closeBottomGate();
        r.p.disengageKicker();
        r.p.enableAutoRotate();
        r.p.disableSort();
        r.p.moveTo(2);
        r.s.setPower(0);
    }

    public void start() {
        schedule(
                Commands.infinite(() -> {
                    r.periodic();
//                    double dist = r.getShootTarget().distanceFrom(r.f.getPose()) + 8;
//                    boolean close = r.f.getPose().getY() > 48;
//                    r.s.forDistance(dist, close);
//                    //r.s.forPose(r.f.getPose(), r.getShootTarget(), close);
////                r.s.setTarget(shootTarget);
////                r.s.setHood(hoodTarget);
//
//                    //  r.t.face(r.getShootTarget(), r.f.getPose());
//                    r.t.face(r.getShootTarget(), r.f.getPose());
//                    r.t.automatic();

                    telemetry.addData("LoopTime Hz", r.getLoopTimeHz());
                    telemetry.addData("Slots", Arrays.toString(r.p.slots));
                    telemetry.addData("Shooter Velocity", r.s.getVelocity());
                    telemetry.addData("Turret Error", r.t.getError());
                    telemetry.addData("Pose", r.f.getPose());
                    telemetry.addData("Target", r.getShootTarget());
                    telemetry.update();
                }),
                Groups.sequential(
                                p.preload(),
                                r.intakeSpindexUnsorted(),
                                p.intakeGate()
//                                r.shootSpindexUnsorted(),
//                                r.intakeSpindexUnsorted(),
//                                p.intakeSpike1()
//                                        .raceWith(Commands.waitMs(3000.0)),
//                                Groups.race(
//                                        p.hitGateAfterFirst()
//                                                .with(
//                                                        Commands.waitMs(500.0)
//                                                                .then(
//                                                                        r.i.off())),
//                                        Commands.waitMs(3000.0)
//                                ),
//                                // Commands.waitMs(),
//                                r.i.in(),
//                                p.scoreHitGate(),
//                                r.shootSpindexUnsorted(),
//                                r.intakeSpindexUnsorted(),
//                                p.intakeSpike2()
//                                        .raceWith(Commands.waitMs(3000.0)),
//
//                                p.scoreSpike2(),
//                                r.shootSpindexUnsorted(),
//                                r.intakeSpindexUnsorted(),
//                                p.intakeSpike3()
//                                        .raceWith(Commands.waitMs(3000.0)),
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
                        .with(
                                Commands.waitMs(29250.0)
                                        .then(
                                                Commands.instant(r.f::breakFollowing),
                                                Commands.instant(() -> r.f.holdPoint(r.f.getPose(), false))
                                        )
                        )
        );
    }

    public void stop() {
        super.stop();
        r.saveEnd();

    }
}
