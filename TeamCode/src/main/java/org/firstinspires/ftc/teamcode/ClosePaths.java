package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.Alliance;

import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.pedro.FollowPath.follow;

public class ClosePaths {
    private final Follower f;
    public final Pose start, score, spike1, spike1Control1, spike1Control2, spike2, spike2Control1, spike2Control2, spike3, spike3Control1, spike3Control2, gateHit, gateHitControl, gateIntake, gateControl1, gateControl2, cornerControl, corner, park;

    public ClosePaths(Robot r, Alliance alliance) {
        this.f = r.follower;

        PoseFactory p = alliance == Alliance.RED ? PoseFactory.degrees().mirrorX(70.75) : PoseFactory.degrees();

        start = p.of(32, 130.5, 90);
        score = p.of(55, 89, 200);

        spike1 = p.of(17.5, 82, 180);
        spike1Control1 = p.of(48, 76,0);
        spike1Control2 = p.of(39.5, 82, 180);

        spike2 = p.of(9.5, 57, 180);
        spike2Control1 = p.of(45, 69,0);
        spike2Control2 = p.of(50, 57, 180);

        spike3 = p.of(10, 36, 180);
        spike3Control1 = p.of(45, 56,0);
        spike3Control2 = p.of(65, 36, 180);

        gateHit = p.of(15, 70, 180);
        gateHitControl = p.of(32, 70, 180);
        gateIntake = p.of(12, 62, 140);
        gateControl1 = p.of(28, 60, 140);
        gateControl2 = p.of(23.25, 45, 0);

        cornerControl = p.of(-5, 30, 0);
        corner = p.of(6.5, 11, 270);

        park = p.of(48, 120, 180);
        
    }

    public CommandBuilder preload() {
        return follow(f, line(start, score).linear(start.heading(), score.heading(), .5));
    }

    public CommandBuilder intakeSpike1() {
        // todo curve
        //        PathChain path = f.pathBuilder().addPath(
        //                        new BezierCurve(
        //                                score,
        //                                spike1Control1,
        //                                spike1Control2,
        //                                spike1
        //                        )
        //                )
        //                .setLinearHeadingInterpolation(score.getHeading(), spike1.getHeading(), .3)
        //                .setBrakingStrength(2)
        //                .build();
        return follow(f, line(score, spike1).linear(score, spike1, .3));
    }

    public CommandBuilder scoreSpike1() {
        // todo curve
        //        PathChain path = f.pathBuilder().addPath(
        //                        new BezierCurve(
        //                                spike1,
        //                                spike1Control2,
        //                                spike1Control1,
        //                                score
        //                        )
        //                ).setLinearHeadingInterpolation(spike1.getHeading(), score.getHeading())
        //                .setBrakingStrength(1.5)
        //                .build();
        return follow(f, line(spike1, score).linear(spike1, score));
    }

    public CommandBuilder intakeSpike2() {
        // todo curve
        //        PathChain path = f.pathBuilder().addPath(
        //                        new BezierCurve(
        //                                score,
        //                                spike2Control1,
        //                                spike2Control2,
        //                                spike2
        //                        ))
        //                .setLinearHeadingInterpolation(score.getHeading(), spike2.getHeading(), .5)
        //                .setBrakingStrength(2)
        //                .build();
        return follow(f, line(score, spike2).linear(score, spike2, .5));
    }

    public CommandBuilder scoreSpike2() {
        // todo curve
        //        PathChain path = f.pathBuilder().addPath(
        //                        new BezierCurve(
        //                                spike2,
        //                                spike2Control2,
        //                                spike2Control1,
        //                                score
        //                        )
        //                ).setLinearHeadingInterpolation(spike2.getHeading(), score.getHeading())
        //                .setBrakingStrength(1.5)
        //                .build();
        return follow(f, line(spike2, score).linear(spike2, score));
    }

    public CommandBuilder intakeSpike3() {
        // todo curve
        //        PathChain path = f.pathBuilder().addPath(
        //                        new BezierCurve(
        //                                score,
        //                                spike3Control1,
        //                                spike3Control2,
        //                                spike3
        //                        ))
        //                .setLinearHeadingInterpolation(score.getHeading(), spike3.getHeading(), .5)
        //                .setBrakingStrength(2)
        //                .setNoDeceleration()
        //                .build();
        return follow(f, line(score, spike3).linear(score, spike3, .5));
    }

    public CommandBuilder scoreSpike3() {
        return follow(f, line(spike3, score).linear(spike3, score));
    }

    public CommandBuilder intakeGate() {
        // todo curve
        //        PathChain path = f.pathBuilder().addPath(
        //                        new BezierCurve(
        //                                score,
        //                                gateControl1,
        //                                gateControl2,
        //                                gateIntake
        //                        )
        //                )
        //                .setLinearHeadingInterpolation(score.getHeading(), gateIntake.getHeading())
        //                .setBrakingStrength(1.5)
        //                .build();
        return follow(f, line(score, gateIntake).linear(score, gateIntake));
    }

    public CommandBuilder scoreGate() {
        // todo curve
        //        PathChain path = f.pathBuilder().addPath(
        //                        new BezierCurve(
        //                                gateIntake,
        //                                gateControl2,
        //                                gateControl1,
        //                                score
        //                        )
        //                )
        //                .setLinearHeadingInterpolation(gateIntake.getHeading(), score.getHeading(), .2)
        //                .setBrakingStrength(2)
        //                .build();
        return follow(f, line(gateIntake, score).linear(gateIntake, score, .2));
    }

    public CommandBuilder hitGateAfterFirst() {
        // todo curve
        //        PathChain path = f.pathBuilder().addPath(
        //                        new BezierCurve(
        //                                spike1,
        //                                gateHitControl,
        //                                gateHit
        //                        )
        //                )
        //                .setLinearHeadingInterpolation(spike1.getHeading(), gateHit.getHeading())
        //                .setBrakingStrength(1.5)
        //                .build();
        return follow(f, line(spike1, gateHit).linear(spike1, gateHit));
    }

    public CommandBuilder hitGateAfterSecond() {
        // todo curve
        //        PathChain path = f.pathBuilder().addPath(
        //                        new BezierCurve(
        //                                spike2,
        //                                gateHitControl,
        //                                gateHit
        //                        )
        //                )
        //                .setLinearHeadingInterpolation(spike2.getHeading(), gateHit.getHeading())
        //                .setBrakingStrength(1.5)
        //                .build();
        return follow(f, line(spike2, gateHit).linear(spike2, gateHit));
    }

    public CommandBuilder scoreHitGate() {
        return follow(f, line(gateHit, score).linear(gateHit, score));
    }

    public CommandBuilder intakeCorner() {
        // todo curve
        //        PathChain path = f.pathBuilder().addPath(
        //                        new BezierCurve(
        //                                score,
        //                                cornerControl,
        //                                corner
        //                        )
        //                )
        //                .setLinearHeadingInterpolation(score.getHeading(), corner.getHeading(), 0.5)
        //                .setBrakingStrength(2)
        //                .build();
        return follow(f, line(score, corner).linear(score, corner, 0.5));
    }

    public CommandBuilder scoreCorner() {
        return follow(f, line(corner, score).linear(corner, score));
    }

    public CommandBuilder park() {
        return follow(f, line(score, park).linear(score, park));
    }

}
