package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.SOTM;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.ftc.FtcLoggingSession;

@Config
public class Close extends CommandOpMode {
    Alliance a;
    ClosePaths p;
    Robot robot;
    MultipleTelemetry telemetryM;
    boolean intakeTime, curr, prev, full;
    Timer intakeTimer = new Timer(), opModeTimer = new Timer();
    double intakeDist;
    public static double tValueToShoot = 1;

    private final FtcLoggingSession psiKit = new FtcLoggingSession();

    public Close(Alliance a) {
        this.a = a;
    }

    public void init() {
        robot = new Robot(hardwareMap, a);
        p = new ClosePaths(robot, a);
        robot.follower.setPose(p.start);
        robot.follower.update();

        robot.shooter.setPower(0);
        robot.turret.face(robot.getShootTarget(), p.score);
        robot.transfer.close();

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        psiKit.start(this, 0);

        Logger.recordMetadata("Alliance", a.name());
        Logger.recordMetadata("OpMode", "Close");

        telemetryM.addData("Pose", robot.follower.pose());
        telemetryM.update();

        schedule(
                Commands.infinite(() -> {
                    robot.turret.face(robot.getAimTarget(), robot.follower.pose());
                    // ── PsiKit per-loop tick ──────────────────────────────────
                    Logger.periodicBeforeUser();
                    psiKit.logOncePerLoop(this);

                    robot.periodic();

                    if (robot.loops % 3 == 0) {
                        intakeDist = robot.intake.getDistance();
                        curr = robot.intake.isDetected(intakeDist);
                    }

                    if (curr && !prev) {
                        intakeTimer.reset();
                    }

                    if (!intakeTime)
                        intakeTime = intakeTimer.s() >= .5;

//                    if (curr && intakeTime) {
//                        if (!robot.shooter.atTarget())
//                            robot.intake.light.orange();
//                        else
//                            robot.intake.light.green();
//                        full = true;
//                    } else {
//                        if (!robot.shooter.atTarget())
//                            robot.intake.light.violet();
//                        else
//                            robot.intake.light.blue();
//                        full = false;
//                    }
                    // TODO LIGHT

                    prev = curr;

                    // ── Log outputs ───────────────────────────────────────────
                    Pose pose = robot.follower.pose();
//                    Pose2D pose2D = PoseConverter.poseToPose2D(pose.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE), InvertedFTCCoordinates.INSTANCE);
//                    Pose2d pose2d = new Pose2d(pose2D.getX(DistanceUnit.METER), pose2D.getY(DistanceUnit.METER), new Rotation2d(pose2D.getHeading(AngleUnit.RADIANS)));
//                    Logger.recordOutput("Pose2d", pose2d);
                    Logger.recordOutput("Pose/X", pose.x());
                    Logger.recordOutput("Pose/Y", pose.y());
                    Logger.recordOutput("Pose/Heading", pose.heading());
                    Logger.recordOutput("Shooter/Velocity", robot.shooter.getVelocity());
                    Logger.recordOutput("Shooter/AtTarget", robot.shooter.atTarget());
                    Logger.recordOutput("Turret/Yaw", robot.turret.getYaw());
//                    Logger.recordOutput("Follower/TValue",       robot.follower.getCurrentTValue());
//                    Logger.recordOutput("Follower/TranslError",  robot.follower.getTranslationalError().toString());
//                    Logger.recordOutput("Follower/HeadingError", robot.follower.getHeadingError());
//                    Logger.recordOutput("Follower/DriveError",   robot.follower.getDriveError());
//                    Logger.recordOutput("Follower/Path", robot.follower.getCurrentPath().toString());
                    Logger.recordOutput("Loop/Hz", robot.getLoopTimeHz());
                    Logger.recordOutput("Intake/Full", full);
                    Logger.recordOutput("Intake/Curr", curr);
                    Logger.recordOutput("Time/OpModeMs", opModeTimer.ms());

                    Logger.periodicAfterUser(0.0, 0.0);
                    // ─────────────────────────────────────────────────────────

                    telemetryM.addData("LoopTime Hz", robot.getLoopTimeHz());
                    telemetryM.addData("Pose", pose);
//                    telemetryM.addData("T Value", robot.follower.getCurrentTValue());
                    telemetryM.addData("Target", robot.getShootTarget());
                    telemetryM.addData("Shooter Velocity", robot.shooter.getVelocity());
                    telemetryM.addData("turret angle", robot.turret.getYaw());
                    telemetryM.addData("full", full);
//                    telemetryM.addData("transl error", robot.follower.getTranslationalError());
//                    telemetryM.addData("heading error", robot.follower.getHeadingError());
//                    telemetryM.addData("drive error", robot.follower.getDriveError());
//                    telemetryM.addData("path", robot.follower.getCurrentPath());
                    telemetryM.update();
                }),
                Groups.sequential(
                        p.preload()
                                .with(
                                        waitUntil(() -> opModeTimer.s() > 1.95)
                                                .then(robot.shoot(p.score))
                                ),
                        spike2(),
                        gate(),
                        gate(),
                        spike1(),
                        gate(),
                        gate(),
                        conditional(
                                () -> opModeTimer.s() < 27.5,
                                gate(),
                                p.park()
                        )
                )
        );
    }

    public void start() {
        robot.shooter.on();
        robot.shooter.forDistance(robot.getShootTarget().distance(p.score), p.score.y() > 48);
        robot.intake.off();
        robot.transfer.off();
        robot.transfer.open();
        robot.turret.face(robot.getShootTarget(), p.score);
        opModeTimer.reset();
    }

    private CommandBuilder spike2() {
        return sequential(
                robot.intake(),
                p.intakeSpike2()
                        .with(
                                waitMs(500)
                                        .then(robot.transfer.closeCommand())
                        )
                        .raceWith(waitMs(3000.0)),
                p.scoreSpike2(),
                robot.shoot(p.score)
        );
    }

    private CommandBuilder spike1() {
        return sequential(
                robot.intake(),
                p.intakeSpike1()
                        .with(
                                waitMs(500)
                                        .then(robot.transfer.closeCommand())
                        )
                        .raceWith(waitMs(2000.0)),
                p.scoreSpike1()
                        .with(
                                waitMs(500.0)
                                        .then(
                                                robot.intake.offCommand(),
                                                robot.transfer.offCommand()
                                        )
                        ),
                robot.shoot(p.score)
        );
    }

    private CommandBuilder gate() {
        return sequential(
                robot.intake(),
                p.intakeGate()
                        .with(
                                waitMs(500)
                                        .then(robot.transfer.closeCommand())
                        )
                        .raceWith(
                                waitMs(2000.0)
                        ),
                waitMs(1750.0)
                        .raceWith(
                                Commands.waitMs(500.0)
                                        .then(Commands.waitUntil(() -> full))
                        ),
                p.scoreGate()
                        .with(
                                waitMs(750.0)
                                        .then(
                                                robot.intake.offCommand(),
                                                robot.transfer.offCommand()
                                        )
                        ),
                robot.intake.offCommand(),
                robot.transfer.offCommand(),
                robot.transfer.openCommand(),
                waitMs(150.0),
                robot.intake(),
                waitMs(100.0)
        );
    }

    public void stop() {
        robot.saveEnd();
        robot.transfer.open();
        psiKit.end();
        super.stop();
    }
}