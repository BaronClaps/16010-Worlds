package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Pattern;

import java.util.List;

import static com.pedropathing.ivy.groups.Groups.sequential;

public class Robot {
    public final Intake i;
    //public final Limelight l;
    public final Shooter s;
    public final Spindexer p;
    public final Turret t;
    public final Follower f;
    public Alliance a;
    public static Pattern currentPattern = Pattern.PGP;

    private final List<LynxModule> hubs;
    private final Timer loop = new Timer();
    public double loops = 0, lastLoop = 0, loopTime = 0;
    public static Pose defaultPose = new Pose(8 + 24, 6.25 + 24, 0);
    public static Pose shootTarget = new Pose(2, 144 - 2, 0);

    public Robot(HardwareMap h, Alliance a) {
        this.a = a;
        i = new Intake(h);
        //l = new Limelight(h, a);
        s = new Shooter(h);
        p = new Spindexer(h);
        t = new Turret(h);
        f = Constants.createFollower(h);

        hubs = h.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        loop.resetTimer();
        setShootTarget();

        periodic();
    }

    public void periodic() {
        setShootTarget();

//        if (loop.getElapsedTime() % 10 == 0) {
//            hub.clearBulkCache();
//        }

        loops++;

        if (loops > 10) {
            double now = loop.getElapsedTime();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        p.periodic();
        f.update();
        t.periodic();
        s.periodic();
    }

    public void saveEnd() {
        defaultPose = f.getPose();
    }


    public void setShootTarget() {
        if (a == Alliance.BLUE && shootTarget.getX() != 2)
            shootTarget = new Pose(2, 144 - 2, 0);
        else if (a == Alliance.RED && shootTarget.getX() != (144 - 2))
            shootTarget = shootTarget.mirror();
    }

    public Pose getShootTarget() {
        return shootTarget;
    }

    public CommandBuilder shootSpindexUnsorted() {
        return sequential(
                i.in(),
                Commands.instant(s::close),
                Commands.waitUntil(s::atTarget),
                i.in(),
                Commands.instant(() -> {
                            p.shootDirection = p.currentIndex > 3 ? -1: 1;
                            p.openBottomGate();
                            p.openTopGate();
                            p.engageKicker();
                            p.all(4);
                        }
                ),
                Commands.wait(1500.0)

        );
    }

    public CommandBuilder shootPassthrough() {
        return sequential(
                i.in(),
                Commands.waitUntil(s::atTarget),
                i.in(),
                Commands.instant(p::openTopGate),
                Commands.wait(350.0)

        );
    }

    public CommandBuilder intakeSorted() {
        return sequential(
                Commands.instant(() -> {
                    p.enableSort();
                    p.disengageKicker();
                    p.openTopGate();
                    p.closeBottomGate();
                }),
                i.in(),
                Commands.wait(250.0)
        );
    }

    public CommandBuilder intakeSpindexUnsorted() {
        return sequential(
                Commands.instant(() -> {
                    p.disableSort();
                    p.enableAutoRotate();
                    p.disengageKicker();
                    p.openTopGate();
                    p.closeBottomGate();
                }),
                i.in()//,
//                Commands.wait(250.0)
        );
    }

    public CommandBuilder intakePassthrough() {
        return sequential(
                Commands.instant(() -> {
                    p.disableAutoRotate();
                    p.disableSort();
                    p.engageKicker();
                    p.closeTopGate();
                    p.openBottomGate();
                }),
                i.in(),
                Commands.wait(250.0)
        );
    }

    public double getLoopTimeMs() {
        return loopTime;
    }

    public double getLoopTimeHz() {
        return 1000 / loopTime;
    }
}