package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.Alliance;

import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.pedro.FollowPath.follow;

public class FarPaths {
    private final Follower f;
    public final Pose start, score, spike3, spike3Control, gateIntake, corner, between, betweenControl, park;

    public FarPaths(Robot r, Alliance alliance) {
        this.f = r.follower;

        PoseFactory p = alliance == Alliance.RED ? PoseFactory.degrees().mirrorX(70.75) : PoseFactory.degrees();

        start = p.of(51.375, 8.188, 180);
        score = p.of(54, 18, 180);

        spike3 = p.of(8, 36, 180);
        spike3Control = p.of(45, 36, 0);
        gateIntake = p.of(7.5, 36, 90);
        corner = p.of(8.25, 9, 180);
        between = p.of(8.25, 30, 90);
        betweenControl = p.of(20, 10, 0);
        park = p.of(36, 24, 180);
    }

    public CommandBuilder preload() {
        return follow(f, line(start, score).linear(start.heading(), score.heading()));
    }

    public CommandBuilder intakeSpike3() {
        // todo curve
        //        PathChain path = f.pathBuilder().addPath(
        //                        new BezierCurve(
        //                                score,
        //                                spike3Control,
        //                                spike3
        //                        )
        //                )
        //                .setLinearHeadingInterpolation(score.getHeading(), spike3.getHeading(), .5)
        //                .setBrakingStrength(1)
        //                .setNoDeceleration()
        //                .build();
        return follow(f, line(score, spike3).linear(score, spike3, .5));
    }

    public CommandBuilder scoreSpike3() {
        return follow(f, line(spike3, score).linear(spike3.heading(), score.heading()));
    }

    public CommandBuilder intakeGate() {
        return follow(f, line(score, gateIntake).linear(score.heading(), gateIntake.heading()));
    }

    public CommandBuilder scoreGate() {
        return follow(f, line(gateIntake, score).linear(gateIntake.heading(), score.heading()));
    }

    public CommandBuilder intakeCorner() {
        return follow(f, line(score, corner).linear(score.heading(), corner.heading(), .25));
    }

    public CommandBuilder scoreCorner() {
        return follow(f, line(corner, score).linear(corner.heading(), score.heading(), .75));
    }

    public CommandBuilder intakeBetween() {
        // todo curve
        //        PathChain path = f.pathBuilder().addPath(
        //                        new BezierCurve(
        //                                score,
        //                                betweenControl,
        //                                between
        //                        )
        //                )
        ////                .setLinearHeadingInterpolation(score.getHeading(), between.getHeading(), .75)
        //                .setTangentHeadingInterpolation()
        //                .setBrakingStrength(1.5)
        //                .build();
        return follow(f, line(score, between).linear(score, between, .75));
    }

    public CommandBuilder scoreBetween() {
        return follow(f, line(between, score).linear(between.heading(), score.heading(), .75));
    }

    public CommandBuilder park() {
        return follow(f, line(score, park).linear(score.heading(), park.heading()));
    }
}
