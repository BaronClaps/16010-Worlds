package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.localization.Localizer;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.List;

import static com.pedropathing.ivy.groups.Groups.sequential;

public class Robot {
    public final Intake intake;
    //public final Limelight l;
    public final Shooter shooter;
    public final Transfer transfer;
    public final Turret turret;
    public final Follower follower;
    public Alliance a;

    private final List<LynxModule> hubs;
    private final Timer loop = new Timer();
    public double loops = 0, lastLoop = 0, loopTime = 0;
    public static Pose defaultPose = new Pose(8 + 24, 6.25 + 24, 0);
    public static Localizer localizer = null;
    public static Pose shootTarget = new Pose(2, 144 - 2, 0);

    public Robot(HardwareMap h, Alliance a) {
        this.a = a;
        intake = new Intake(h);
        shooter = new Shooter(h);
        transfer = new Transfer(h);
        turret = new Turret(h);

        follower = Constants.createFollower(h);

        hubs = h.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        loop.resetTimer();
        setShootTarget();

        periodic();
    }

    public void periodic() {
        loops++;

        if (loops > 10) {
            double now = loop.getElapsedTime();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        follower.update();
        shooter.periodic();
    }

    public void saveEnd() {
        defaultPose = follower.getPose();
        localizer = follower.getPoseTracker().getLocalizer();
    }


    public void setShootTarget() {
        if (a == Alliance.BLUE && shootTarget.getX() != 2)
            shootTarget = new Pose(2, 144 - 2, 0);
        else if (a == Alliance.RED && shootTarget.getX() != (144 - 2))
            shootTarget = new Pose(2, 144 - 2, 0).mirror();
    }

    public Pose getShootTarget() {
        return shootTarget;
    }

    public CommandBuilder shootPassthrough() {
        return sequential(
                intake.inCommand(),
                Commands.instant(shooter::close),
                Commands.instant(turret::on),
                Commands.waitUntil(shooter::atTarget),
                Commands.waitUntil(turret::isReady),
                intake.inCommand(),
                Commands.instant(transfer::openTopGate),
                Commands.instant(transfer::engageKicker),
                Commands.waitMs(350.0),
                Commands.instant(turret::off)

        );
    }

    public CommandBuilder intakePassthrough() {
        return sequential(
                Commands.instant(() -> {
                    transfer.disableAutoRotate();
                    transfer.disableSort();
                    transfer.disengageKicker();
                    transfer.closeTopGate();
                    transfer.openBottomGate();
                    turret.on();
                }),
                intake.inCommand(),
                Commands.waitMs(250.0)
        );
    }

    public double getLoopTimeMs() {
        return loopTime;
    }

    public double getLoopTimeHz() {
        return 1000 / loopTime;
    }
}