package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class Paths {
    private final Follower f;

    public Pose start = new Pose(33.5, 135.125, Math.toRadians(270));
    public Pose startMid = start.withY(100);
    public Pose score = new Pose(60, 76, Math.toRadians(180));

    public Pose spike1 = new Pose(17, 85, Math.toRadians(180));
    public Pose spike1Control1 = new Pose(48, 79);
    public Pose spike1Control2 = spike1.withX(39.5);
    
    public Pose spike2 = new Pose(10, 60, Math.toRadians(180));
    public Pose spike2Control1 = new Pose(45, 80);
    public Pose spike2Control2 = spike2.withX(65);

    public Pose gateIntake = new Pose(10, 58.75+1, Math.toRadians(110));
    public Pose gateControl1 = new Pose(48, 79);
    public Pose gateControl2 = new Pose(23.25, 47);

    public Pose park = new Pose(36, 72, Math.toRadians(180));

    public Paths(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.RED)) {
            start = start.mirror();
            startMid = startMid.mirror();
            score = score.mirror();

            spike1 = spike1.mirror();
            spike1Control1 = spike1Control1.mirror();
            spike1Control2 = spike1Control2.mirror();

            spike2 = spike2.mirror();
            spike2Control1 = spike2Control1.mirror();
            spike2Control2 = spike2Control2.mirror();

            gateIntake = gateIntake.mirror();
            gateControl1 = gateControl1.mirror();
            gateControl2 = gateControl2.mirror();

            park = park.mirror();
        }
    }

    public CommandBuilder preload() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                start,
                                startMid,
                                score
                        )
                )
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();
        return PedroCommands.follow(this.f, path);
    }

    public CommandBuilder intakeSpike1() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                score,
                                spike1Control1,
                                spike1Control2,
                                spike1
                        )
                ).setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();
        return PedroCommands.follow(this.f, path);
    }

    public CommandBuilder scoreSpike1() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                spike1,
                                spike1Control2,
                                spike1Control1,
                                score
                        )
                ).setLinearHeadingInterpolation(spike1.getHeading(), score.getHeading())
                .setReversed()
                .build();
        return PedroCommands.follow(this.f, path);
    }

    public CommandBuilder intakeSpike2() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                score,
                                spike2Control1,
                                spike2Control2,
                                spike2
                        )
                ).setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();
        return PedroCommands.follow(this.f, path);
    }

    public CommandBuilder scoreSpike2() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                spike2,
                                spike2Control2,
                                spike2Control1,
                                score
                        )
                ).setLinearHeadingInterpolation(spike2.getHeading(), score.getHeading())
                .setReversed()
                .build();
        return PedroCommands.follow(this.f, path);
    }

    public CommandBuilder intakeGate() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                score,
                                gateControl1,
                                gateControl2,
                                gateIntake
                        )
                ).setLinearHeadingInterpolation(score.getHeading(), gateIntake.getHeading())
                .build();
        return PedroCommands.follow(this.f, path);
    }

    public CommandBuilder scoreGate() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                gateIntake,
                                gateControl2,
                                gateControl1,
                                score
                        )
                ).setLinearHeadingInterpolation(gateIntake.getHeading(), score.getHeading())
                .setReversed()
                .build();
        return PedroCommands.follow(this.f, path);
    }

    public CommandBuilder park() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                score,
                                park
                        )
                ).setLinearHeadingInterpolation(score.getHeading(), park.getHeading())
                .build();
        return PedroCommands.follow(this.f, path);
    }
}
