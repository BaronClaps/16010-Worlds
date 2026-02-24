package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BaronPose;

public class Paths {
    private final Follower f;

    public Pose start = new Pose(31.3125, 144-11, Math.toRadians(90));
    public Pose startMid = start.withY(100);
    public Pose score = new Pose(55, 144-55, Math.toRadians(135));

    public Pose spike1 = new Pose(17.5, 85, Math.toRadians(180));
    public Pose spike1Control1 = new Pose(48, 79);
    public Pose spike1Control2 = spike1.withX(39.5);
    
    public Pose spike2 = new Pose(10, 60, Math.toRadians(180));
    public Pose spike2Control1 = new Pose(45, 80);
    public Pose spike2Control2 = spike2.withX(65);

    public Pose spike3 = new Pose(10, 60-24, Math.toRadians(180));
    public Pose spike3Control1 = new Pose(45, 80-24);
    public Pose spike3Control2 = spike3.withX(65);

    public Pose gateHit = new Pose (15, 74, Math.toRadians(180));
    public Pose gateHitControl = gateHit.withX(32);

    public Pose gateIntake = new Pose(10, 58.75+1, Math.toRadians(110));
    public Pose gateControl1 = new Pose(48, 79);
    public Pose gateControl2 = new Pose(23.25, 47);

    public Pose cornerControl = new Pose(8, 30);
    public Pose corner = new Pose(8, 11, Math.toRadians(270));

    public Pose park = new Pose(36, 72, Math.toRadians(180));

    public Paths(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.RED)) {
            start = BaronPose.mirror(start);
            startMid = BaronPose.mirror(startMid);
            score = BaronPose.mirror(score);

            spike1 = BaronPose.mirror(spike1);
            spike1Control1 = BaronPose.mirror(spike1Control1);
            spike1Control2 = BaronPose.mirror(spike1Control2);

            spike2 = BaronPose.mirror(spike2);
            spike2Control1 = BaronPose.mirror(spike2Control1);
            spike2Control2 = BaronPose.mirror(spike2Control2);

            spike3 = BaronPose.mirror(spike3);
            spike3Control1 = BaronPose.mirror(spike3Control1);
            spike3Control2 = BaronPose.mirror(spike3Control2);

            gateIntake = BaronPose.mirror(gateIntake);
            gateControl1 = BaronPose.mirror(gateControl1);
            gateControl2 = BaronPose.mirror(gateControl2);

            gateHit = BaronPose.mirror(gateHit);
            gateHitControl = BaronPose.mirror(gateHitControl);

            cornerControl = BaronPose.mirror(cornerControl);
            corner = BaronPose.mirror(corner);

            park = BaronPose.mirror(park);
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
                )
                .setLinearHeadingInterpolation(score.getHeading(), spike1.getHeading(), .3)
                .setBrakingStrength(2)
                .build();
        return PedroCommands.follow(this.f, path);
    }

    public CommandBuilder scoreSpike1() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                spike1,
//                                spike1Control2,
//                                spike1Control1,
                                score
                        )
                ).setLinearHeadingInterpolation(spike1.getHeading(), score.getHeading())
                .setBrakingStrength(1.5)
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
                        ))
                .setLinearHeadingInterpolation(score.getHeading(), spike2.getHeading(), .5)
                .setBrakingStrength(2)
                .build();
        return PedroCommands.follow(this.f, path);
    }

    public CommandBuilder scoreSpike2() {
        PathChain path = f.pathBuilder().addPath(
                        new BezierLine(
                                spike2,
//                                spike2Control2,
//                                spike2Control1,
                                score
                        )
                ).setLinearHeadingInterpolation(spike2.getHeading(), score.getHeading())
                .setBrakingStrength(1.5)
                .build();
        return PedroCommands.follow(this.f, path);
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
                .build();
        return PedroCommands.follow(this.f, path);
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
                .setBrakingStrength(1.5)
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
                .setBrakingStrength(1.5)
                .build();
        return PedroCommands.follow(this.f, path);
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
        return PedroCommands.follow(this.f, path);
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
        return PedroCommands.follow(this.f, path);
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
        return PedroCommands.follow(this.f, path);
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
        return PedroCommands.follow(this.f, path);
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
