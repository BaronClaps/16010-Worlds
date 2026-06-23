package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.FollowPath;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class ClosePaths {
    private final Follower f;

    public Pose start = new Pose(32, 130.5, Math.toRadians(90));
    public Pose score = new Pose(55, 89, Math.toRadians(200));

    public Pose spike1 = new Pose(17.5, 82, Math.toRadians(180));
    public Pose spike1Control1 = new Pose(48, 76);
    public Pose spike1Control2 = spike1.withX(39.5);
    
    public Pose spike2 = new Pose(9.5, 57, Math.toRadians(180));
    public Pose spike2Control1 = new Pose(45, 69);
    public Pose spike2Control2 = spike2.withX(50);

    public Pose spike3 = new Pose(10, 36, Math.toRadians(180));
    public Pose spike3Control1 = new Pose(45, 56);
    public Pose spike3Control2 = spike3.withX(65);

    public Pose gateHit = new Pose(15, 70, Math.toRadians(180));
    public Pose gateHitControl = gateHit.withX(32);
    public Pose gateIntake = new Pose(12, 62, Math.toRadians(140));
    public Pose gateControl1 = new Pose(28, 60, Math.toRadians(140));
    public Pose gateControl2 = new Pose(23.25, 45);

    public Pose cornerControl = new Pose(-5, 30);
    public Pose corner = new Pose(6.5, 11, Math.toRadians(270));

    public Pose park = new Pose(48, 120, Math.toRadians(180));

    public ClosePaths(Robot r, Alliance alliance) {
        this.f = r.follower;

        PoseFactory.degrees().mirrorX(141.5);
    }

    public CommandBuilder preload() {
//        PathChain path = f.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                start,
//                                score
//                        )
//                )
//                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading(), .5)
//                .setBrakingStrength(2)
//                .build();
//        return new FollowPath(this.f, path, .95);
        return new FollowPath(this.f, Paths.line(start, score).linear(start.heading(), score.heading()));
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
                .setBrakingStrength(1.5)
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
