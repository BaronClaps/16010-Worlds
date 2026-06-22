package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedro.FollowPath;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class FarPaths {
    private final Follower f;
    Alliance a = Alliance.BLUE;

    public Pose start = new Pose(51.375, 8.188, Math.PI);
    public Pose score = new Pose(72 - 18, 18, Math.toRadians(180));

    public Pose spike3 = new Pose(8, 36, Math.toRadians(180));
    public Pose spike3Control = new Pose(45, 36);
    public Pose gateIntake = new Pose(7.5, 36, Math.toRadians(90));
    public Pose corner = new Pose(8.25, 9, Math.toRadians(180));
    public Pose between = new Pose(8.25, 30, Math.toRadians(90));
    public Pose betweenControl = new Pose(20, 10);
    public Pose park = new Pose(36, 24, Math.toRadians(180));

    public FarPaths(Robot r) {
        this.f = r.follower;

        if (!r.alliance.equals(a)) {
            start = start.mirror();
            score = score.mirror();

            spike3 = spike3.mirror();
            spike3Control = spike3Control.mirror();

            gateIntake = gateIntake.mirror();
            between = between.mirror();
            betweenControl = betweenControl.mirror();
            corner = corner.mirror();
            park = park.mirror();

            a = r.alliance;
        }
    }

    public CommandBuilder preload() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                start,
                                score
                        )
                )
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();

        return new FollowPath(this.f, path, .95);
    }

    public CommandBuilder intakeSpike3() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                score,
                                spike3Control,
                                spike3
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), spike3.getHeading(), .5)
                .setBrakingStrength(1)
                .setNoDeceleration()
                .build();
        return new FollowPath(this.f, path);
    }

    public CommandBuilder scoreSpike3() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                spike3,
                                score
                        )
                )
                .setLinearHeadingInterpolation(spike3.getHeading(), score.getHeading())
                .setBrakingStrength(2)
//                .setNoDeceleration()
                .build();
        return new FollowPath(this.f, path, .95);
    }

    public CommandBuilder intakeGate() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                score,
                                gateIntake
                        )
                ).setLinearHeadingInterpolation(score.getHeading(), gateIntake.getHeading())
                .setBrakingStrength(2)
                .build();
        return new FollowPath(this.f, path);
    }

    public CommandBuilder scoreGate() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                gateIntake,
                                score
                        )
                ).setLinearHeadingInterpolation(gateIntake.getHeading(), score.getHeading())
                .setBrakingStrength(2)
                .build();
        return new FollowPath(this.f, path, .95);
    }

    public CommandBuilder intakeCorner() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                score,
                                corner
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), corner.getHeading(), .25)
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
                .setLinearHeadingInterpolation(corner.getHeading(), score.getHeading(), .75)
                .setBrakingStrength(2)
                .build();
        return new FollowPath(this.f, path, .95);
    }

    public CommandBuilder intakeBetween() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierCurve(
                                score,
                                betweenControl,
                                between
                        )
                )
//                .setLinearHeadingInterpolation(score.getHeading(), between.getHeading(), .75)
                .setTangentHeadingInterpolation()
                .setBrakingStrength(1.5)
                .build();
        return new FollowPath(this.f, path);
    }

    public CommandBuilder scoreBetween() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                between,
                                score
                        )
                )
                .setLinearHeadingInterpolation(between.getHeading(), score.getHeading(), .75)
                .setBrakingStrength(2)
                .build();
        return new FollowPath(this.f, path, .95);
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
