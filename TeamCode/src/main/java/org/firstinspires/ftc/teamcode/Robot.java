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

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
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
    public static Pose shootTarget = new Pose(0, 141.5, 0);
    public static Pose aimTarget = new Pose(2, 141.5 - 2, 0);

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
        if (alliance == Alliance.BLUE)
            shootTarget = new Pose(0, 141.5, 0);
        else if (alliance == Alliance.RED)
            shootTarget = new Pose(0, 141.5, 0).mirror();

        if (alliance == Alliance.BLUE)
            aimTarget = new Pose(0, 141.5, 0);
        else if (alliance == Alliance.RED)
            aimTarget = new Pose(0, 141.5, 0).mirror();
    }

    public Pose getShootTarget() {
        return shootTarget;
    }
    public Pose getAimTarget() {
        return aimTarget;
    }

    public CommandBuilder shoot(Pose score) {
        return sequential(
                instant(() -> shooter.forDistance(getShootTarget().distanceFrom(score), score.getY() > 48)),
//                instant(() -> shooter.setTarget(1300)),
//                instant(() -> intake.set(-.01)),
//                instant(() -> transfer.set(-.01)),
//                waitMs(350.0),
                intake.inCommand(),
                transfer.setCommand(0.5),
                Commands.waitUntil(shooter::atTarget),
                transfer.openCommand(),
//                waitMs(300.0),
                intake.inCommand(),
                transfer.inCommand()
        )
                .raceWith(
                        Commands.infinite(() -> {
//                            Pose predicted = Turret.getPredictedPose(follower.getPose(), getShootTarget(), follower.getVelocity(), follower.getAngularVelocity());
                            turret.face(getAimTarget(), follower.getPose());
                            shooter.forDistance(getShootTarget().distanceFrom(score), score.getY() > 48);
                           // shooter.forDistance(getShootTarget().distanceFrom(predicted), predicted.getY() > 48);
                        }));
    }

    public CommandBuilder shootNoSOTM(Pose score) {
        return sequential(
                instant(() -> shooter.forDistance(getShootTarget().distanceFrom(score), score.getY() > 48)),
                instant(() -> turret.face(getAimTarget(), score)),
                intake.inCommand(),
                transfer.setCommand(0.5),
                Commands.waitUntil(shooter::atTarget),
                transfer.openCommand(),
                intake.inCommand(),
                transfer.inCommand(),
                waitMs(500.0),
                transfer.closeCommand()
        );
    }

    public CommandBuilder shootNoSOTMFar(Pose score) {
        return sequential(
                instant(() -> shooter.forDistance(getShootTarget().distanceFrom(score), score.getY() > 48)),
                instant(() -> turret.face(getAimTarget(), score)),
                Commands.waitUntil(shooter::atTarget),
                instant(() -> intake.set(.7)),
                instant(() -> transfer.set(.7)),
                waitMs(250.0),
                transfer.openCommand(),
                waitMs(750.0),
                transfer.closeCommand()
        )
                .raceWith(
                        Commands.infinite(() -> {
                            turret.face(getAimTarget(), follower.getPose());
                        }));
    }

    public CommandBuilder intake() {
        return sequential(
//                transfer.closeCommand(),
                transfer.setCommand(0.15),
                intake.inCommand()
        );
    }

    public double getLoopTimeMs() {
        return loopTime;
    }

    public double getLoopTimeHz() {
        return 1000 / loopTime;
    }
}