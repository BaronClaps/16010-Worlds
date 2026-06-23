//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.PathChain;

public class FollowPath extends CommandBuilder {
    private final Follower follower;
    private final PathChain path;
    private double tConstraint = .975;
    private boolean holdEnd;
    private double maxPower;

    public FollowPath(Follower f, PathChain pathChain) {
        this.follower = f;
        this.path = pathChain;
        this.maxPower = this.follower.getMaxPowerScaling();
        this.holdEnd = this.follower.constants.automaticHoldEnd;
        this.initialize();
    }

    public FollowPath(Follower f, PathChain pathChain, double tConstraint) {
        this(f, pathChain);
        this.tConstraint = tConstraint;
    }

//    public FollowPath(Follower f, PathChain pathChain, double maxPower) {
//        this.follower = f;
//        this.path = pathChain;
//        this.maxPower = maxPower;
//        this.holdEnd = this.follower.constants.automaticHoldEnd;
//        this.initialize();
//    }

    public FollowPath(Follower f, PathChain pathChain, boolean holdEnd) {
        this.follower = f;
        this.path = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = this.follower.getMaxPowerScaling();
        this.initialize();
    }

    public FollowPath(Follower f, PathChain pathChain, boolean holdEnd, double maxPower) {
        this.follower = f;
        this.path = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
        this.initialize();
    }

    private void initialize() {
        this.setStart(() -> this.follower.followPath(this.path, this.maxPower, this.holdEnd));
        this.setDone(() -> this.follower.getCurrentTValue() > tConstraint);
        //this.setDone(() -> !this.follower.isBusy());
    }
}
