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

        schedule(
               Commands.infinite(() -> {
                   r.periodic();
//                   double dist = r.getShootTarget().distanceFrom(r.f.getPose()) + 10;
//                   boolean close = r.f.getPose().getY() > 48;
//                   r.s.forDistance(dist, close);
                   //r.s.forPose(r.f.getPose(), r.getShootTarget(), close);
//                r.s.setTarget(shootTarget);
//                r.s.setHood(hoodTarget);

//                   r.t.face(r.getShootTarget(), r.f.getPose());
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
                        Commands.instant(() -> r.t.face(r.getShootTarget(), p.score)),
                        p.preload(),
                        r.shootSpindexUnsorted(),
                        r.intakeSpindexUnsorted(),
                        p.intakeSpike2(),
                        p.scoreSpike2(),
                        r.shootSpindexUnsorted(),
                        p.intakeGate(),
                        r.intakeSpindexUnsorted(),
                        Commands.wait(2000.0),
                        p.scoreGate(),
                        r.shootSpindexUnsorted(),
                        p.intakeGate(),
                        r.intakeSpindexUnsorted(),
                        Commands.wait(2000.0),
                        p.scoreGate(),
                        r.shootSpindexUnsorted(),
                        r.intakeSpindexUnsorted(),
                        p.intakeSpike1(),
                        p.scoreSpike1(),
                        r.shootSpindexUnsorted(),
                        p.park()
                )
        );
    }
}
