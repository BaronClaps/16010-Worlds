package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

import java.util.Arrays;

import static com.pedropathing.ivy.groups.Groups.sequential;

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
    }

        public void start() {
        schedule(
               Commands.infinite(() -> {
                   r.periodic();
//                   double dist = r.getShootTarget().distanceFrom(r.f.getPose()) + 10;
//                   boolean close = r.f.getPose().getY() > 48;
//                   r.s.forDistance(dist, close);
                   //r.s.forPose(r.f.getPose(), r.getShootTarget(), close);
//                r.s.setTarget(shootTarget);
//                r.s.setHood(hoodTarget);

                 //  r.t.face(r.getShootTarget(), r.f.getPose());
                   r.t.face(r.getShootTarget(), p.score);
                   r.t.automatic();

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
                        r.shootPassthrough(),
                        r.intakeSpindexUnsorted(),
                        p.intakeSpike1()
                                .raceWith(Commands.wait(3000.0)),
                        Groups.race(
                        p.hitGate()
                                        .with(
                                                Commands.wait(500.0)
                                                        .then(
                                                                r.i.off())),
                                Commands.wait(3000.0)
                        ),
                       // Commands.wait(),
                        r.i.in(),
                        p.scoreHitGate(),
                        r.shootSpindexUnsorted(),
                        r.intakeSpindexUnsorted(),
                        p.intakeSpike2()
                                .raceWith(Commands.wait(3000.0)),
                        p.scoreSpike2(),
                        r.shootSpindexUnsorted(),
                        r.intakeSpindexUnsorted(),
                        p.intakeSpike3()
                                .raceWith(Commands.wait(3000.0)),
                        p.scoreSpike3(),
                        r.shootSpindexUnsorted(),
                        r.intakeSpindexUnsorted(),
                        p.intakeCorner()
                                .raceWith(Commands.wait(6000.0)),
                        p.scoreCorner(),
                        r.shootSpindexUnsorted(),
                        p.park(),
                        Commands.instant(r.s::off),
                        r.i.off()
                )
        );
    }

    public void stop() {
        super.stop();
        r.saveEnd();

    }
}
