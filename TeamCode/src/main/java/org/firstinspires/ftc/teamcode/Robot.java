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
    public Alliance alliance;

    private final Timer loop = new Timer();
    public double loops = 0, lastLoop = 0, loopTime = 0;
    public static Pose defaultPose = new Pose(8 + 24, 6.25 + 24, 0);
    public static Localizer localizer = null;
    public static Pose shootTarget = new Pose(2, 144 - 2, 0);

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
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
        if (alliance == Alliance.BLUE && shootTarget.getX() != 2)
            shootTarget = new Pose(2, 144 - 2, 0);
        else if (alliance == Alliance.RED && shootTarget.getX() != (144 - 2))
            shootTarget = new Pose(2, 144 - 2, 0).mirror();
    }

    public Pose getShootTarget() {
        return shootTarget;
    }

    public CommandBuilder shoot(Pose score) {
        return sequential(
                Commands.instant(shooter::close),
                Commands.instant(() -> turret.face(getShootTarget(), score)),
                Commands.instant(() -> intake.set(-.00001)),
                Commands.instant(() -> transfer.set(-.00001)),
                intake.lowerCommand(),
                Commands.waitMs(100.0),
                transfer.openCommand(),
                Commands.waitMs(250.0),
                Commands.waitUntil(shooter::atTarget),
                intake.inCommand(),
                transfer.inCommand(),
                Commands.waitMs(500.0),
                transfer.closeCommand(),
                intake.lowerCommand()
        );
    }

    public CommandBuilder intake() {
        return sequential(
                transfer.closeCommand(),
                transfer.inCommand(),
                intake.inCommand(),
                intake.lowerCommand(),
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