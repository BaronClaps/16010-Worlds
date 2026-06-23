//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.Path;

public class FollowPath extends CommandBuilder {
    private final Follower follower;
    private final Path path;

    public FollowPath(Follower f, Path path) {
        this.follower = f;
        this.path = path;
    }

    private void initialize() {
        this.setStart(() -> this.follower.follow(this.path));
        this.setDone(() -> this.follower.mode() != Follower.Mode.FOLLOW);
    }
}
