package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.FarPaths.start;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedro.FollowPath;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class ClosePaths {
    private final Follower f;
    Alliance a = Alliance.BLUE;

    public static Pose start = new Pose(24+8.5, 144-8.375, Math.toRadians(90));
    public Pose score = new Pose(55, 144-55, Math.toRadians(180+20));

    public Pose spike1 = new Pose(17.5, 85, Math.toRadians(180));
    public Pose spike1Control1 = new Pose(48, 79);
    public Pose spike1Control2 = spike1.withX(39.5);
    
    public Pose spike2 = new Pose(8, 60, Math.toRadians(180));
    public Pose spike2Control1 = new Pose(45, 72);
    public Pose spike2Control2 = spike2.withX(50);

    public Pose spike3 = new Pose(10, 60-24, Math.toRadians(180));
    public Pose spike3Control1 = new Pose(45, 80-24);
    public Pose spike3Control2 = spike3.withX(65);

    public Pose gateHit = new Pose (15, 74, Math.toRadians(180));
    public Pose gateHitControl = gateHit.withX(32);

    //public Pose gateIntake = new Pose(133.5-2, 60.37+2.5, Math.toRadians(20)).mirror();
    public Pose gateIntake = new Pose(131.5, 63, Math.toRadians(30)).mirror();
    public Pose gateControl1 = new Pose(48, 70);
    public Pose gateControl2 = new Pose(23.25, 65);

    public Pose cornerControl = new Pose(-5, 30);
    public Pose corner = new Pose(6.5, 11, Math.toRadians(270));

    public Pose park = new Pose(48, 144-24, Math.toRadians(180));

    public ClosePaths(Robot r) {
        this.f = r.follower;

        if (!r.alliance.equals(a)) {
            start = start.mirror();
            score = score.mirror();

            spike1 = spike1.mirror();
            spike1Control1 = spike1Control1.mirror();
            spike1Control2 = spike1Control2.mirror();

            spike2 = spike2.mirror();
            spike2Control1 = spike2Control1.mirror();
            spike2Control2 = spike2Control2.mirror();

            spike3 = spike3.mirror();
            spike3Control1 = spike3Control1.mirror();
            spike3Control2 = spike3Control2.mirror();

            gateIntake = gateIntake.mirror();
            gateControl1 = gateControl1.mirror();
            gateControl2 = gateControl2.mirror();

            gateHit = gateHit.mirror();
            gateHitControl = gateHitControl.mirror();

            cornerControl = cornerControl.mirror();
            corner = corner.mirror();

            park = park.mirror();
            a = r.alliance;
        }
    }

    public CommandBuilder preload() {
        PathChain path = f.pathBuilder()
                .addPath(
                        new BezierLine(
                                start,
                                score
                        )
                )
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(0, .25, HeadingInterpolator.tangent.reverse()),
//                                new HeadingInterpolator.PiecewiseNode(.25, 1, HeadingInterpolator.linear(start.getHeading(), score.getHeading())
//                                )
//                        )
//                )
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading(), .5)
                .setBrakingStrength(2)
                .build();
        return new FollowPath(this.f, path, .95);
    }

    public CommandBuilder intakeSpike1() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                score,
                                spike1Control1,
                                spike1Control2,
                                spike1
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), spike1.getHeading(), .3)
                .setBrakingStrength(2)
//                .setNoDeceleration()
                .build();
        return new FollowPath(this.f, path);
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
                .setBrakingStrength(1.5)
                .build();
        return new FollowPath(this.f, path, .95);
    }

    public CommandBuilder intakeSpike2() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                score,
                                spike2Control1,
                                spike2Control2,
                                spike2
                        ))
                .setLinearHeadingInterpolation(score.getHeading(), spike2.getHeading(), .5)
                .setBrakingStrength(2)
                .build();
        return new FollowPath(this.f, path);
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
                .setBrakingStrength(2)
                .build();
        return new FollowPath(this.f, path, .95);
    }

    public CommandBuilder intakeSpike3() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                score,
                                spike3Control1,
                                spike3Control2,
                                spike3
                        ))
                .setLinearHeadingInterpolation(score.getHeading(), spike3.getHeading(), .5)
                .setBrakingStrength(2)
                .setNoDeceleration()
                .build();
        return new FollowPath(this.f, path);
    }

    public CommandBuilder scoreSpike3() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                spike3,
//                                spike2Control2,
//                                spike2Control1,
                                score
                        )
                ).setLinearHeadingInterpolation(spike3.getHeading(), score.getHeading())
                .setBrakingStrength(1.5)
                .setNoDeceleration()
                .build();
        return new FollowPath(this.f, path);
    }

    public CommandBuilder intakeGate() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                score,
                                gateControl1,
                                gateControl2,
                                gateIntake
                        )
                )
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(0, .25, HeadingInterpolator.tangent),
//                                new HeadingInterpolator.PiecewiseNode(.25, 1, HeadingInterpolator.linear(score.getHeading(), gateIntake.getHeading())
//                                )
//                        )
//                )
                .setLinearHeadingInterpolation(score.getHeading(), gateIntake.getHeading())
                .setBrakingStrength(1.5)
                .build();
        return new FollowPath(this.f, path);
    }

    public CommandBuilder scoreGate() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                gateIntake,
                                gateControl2,
                                gateControl1,
                                score
                        )
                )
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(0, .5, HeadingInterpolator.linear(gateIntake.getHeading(), score.getHeading())),
//                                new HeadingInterpolator.PiecewiseNode(.5, 1, HeadingInterpolator.tangent.reverse())
//                        )
//                )
                .setLinearHeadingInterpolation(gateIntake.getHeading(), score.getHeading(), .2)
                .setBrakingStrength(2)
                .build();
        return new FollowPath(this.f, path, .95);
    }

    public CommandBuilder hitGateAfterFirst() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                spike1,
                                gateHitControl,
                                gateHit
                        )
                )
                .setLinearHeadingInterpolation(spike1.getHeading(), gateHit.getHeading())
                .setBrakingStrength(1.5)
                .build();
        return new FollowPath(this.f, path);
    }

    public CommandBuilder hitGateAfterSecond() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                spike2,
                                gateHitControl,
                                gateHit
                        )
                )
                .setLinearHeadingInterpolation(spike2.getHeading(), gateHit.getHeading())
                .setBrakingStrength(1.5)
                .build();
        return new FollowPath(this.f, path);
    }

    public CommandBuilder scoreHitGate() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                gateHit,
                                score
                        )
                )
                .setLinearHeadingInterpolation(gateHit.getHeading(), score.getHeading())
                .setBrakingStrength(1.5)
                .build();
        return new FollowPath(this.f, path);
    }

    public CommandBuilder intakeCorner() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                score,
                                cornerControl,
                                corner
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), corner.getHeading(), 0.5)
                .setBrakingStrength(2)
                .build();
        return new FollowPath(this.f, path);
    }

    public CommandBuilder scoreCorner() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                corner,
                                score
                        )
                )
                .setLinearHeadingInterpolation(corner.getHeading(), score.getHeading())
                .setBrakingStrength(2)
                .build();
        return new FollowPath(this.f, path);
    }

    public CommandBuilder park() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                score,
                                park
                        )
                ).setLinearHeadingInterpolation(score.getHeading(), park.getHeading())
                .build();
        return new FollowPath(this.f, path);
    }

}
