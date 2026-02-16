package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BaronPose;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Robot.defaultPose;
import static org.firstinspires.ftc.teamcode.subsystem.Spindexer.*;

@Config
public class Tele extends CommandOpMode {
    enum IndexMode {
        PASSTHROUGH, UNSORTED, SORTED
    }

    Robot r;
    final Alliance a;

    public boolean shoot = false, manual = false, field = true, wasFull = false, wasDone = true;
    public IndexMode indexMode = IndexMode.UNSORTED;
    public double intakeOn = 0, speed = 1;
    public static double shootTarget = 1650, hoodTarget = 0.55;

    public Tele(Alliance alliance) {
        a = alliance;
    }

    @Override
    public void init() {
        r = new Robot(hardwareMap, a);
        r.f.setStartingPose(defaultPose);
        r.t.setPowerZero();
        r.p.setPattern(Robot.currentPattern);
//        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed())
            r.t.resetTurret();
    }

    @Override
    public void start() {
        r.setShootTarget();
        r.periodic();
        r.t.setPowerZero();
        r.f.startTeleopDrive();
        r.i.spinOff();
        r.p.reset();
        r.p.closeTopGate();
        r.p.closeBottomGate();
        r.p.disengageKicker();
        r.p.enableAutoRotate();
        r.p.disableSort();
        r.s.setHood(.5);
    }

    @Override
    public void loop() {
        r.periodic();

            if (field)
                r.f.setTeleOpDrive(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x, speed * -gamepad1.right_stick_x, false, r.a == Alliance.BLUE ? Math.toRadians(180) : 0);
            else
                r.f.setTeleOpDrive(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x, speed * -gamepad1.right_stick_x, true);

        if (gamepad1.rightBumperWasPressed())
            if (intakeOn == 1)
                intakeOn = 0;
            else
                intakeOn = 1;

        if (gamepad1.dpadDownWasPressed())
            if (intakeOn == 2)
                intakeOn = 0;
            else
                intakeOn = 2;

        if (intakeOn == 1)
            r.i.spinIn();
        else if (intakeOn == 2)
            r.i.spinOut();
        else
            r.i.spinOff();

        if (shoot) {
            r.s.on();
            r.t.on();

            if (manual) {
                r.t.manual(-gamepad1.right_trigger + gamepad1.left_trigger);
                r.s.setTarget(shootTarget);
                r.s.setHood(hoodTarget);
            } else {
                double dist = r.getShootTarget().distanceFrom(r.f.getPose()) + 10;
                boolean close = r.f.getPose().getY() > 48;
                r.s.forDistance(dist, close);
                //r.s.forPose(r.f.getPose(), r.getShootTarget(), close);
//                r.s.setTarget(shootTarget);
//                r.s.setHood(hoodTarget);
                r.t.face(r.getShootTarget(), r.f.getPose());
                r.t.automatic();
            }
        } else {
            r.s.off();
            r.t.off();
        }

        if ((gamepad1.aWasPressed() || gamepad1.leftBumperWasPressed()) && shoot) {
            r.i.spinIn();
            intakeOn = 1;
            r.p.engageKicker();
            r.p.openTopGate();
            r.p.openBottomGate();

            if (indexMode == IndexMode.UNSORTED) {
                r.p.shootDirection = r.p.currentIndex >= 3 ? -1 : 1;
                r.p.all(4);
            } else if (indexMode == IndexMode.SORTED) {
                //TODO fix ts
                r.p.shootDirection = r.p.currentIndex >= 3 ? -1 : 1;
                r.p.all(3);
            } else {
                r.p.all(0);
            }
        }

        if ((r.p.done.getElapsedTimeSeconds() > 0.25 && !wasDone && !shooting) || gamepad1.optionsWasPressed()){
            intakeOn = 1;
            r.i.spinIn();
            r.p.disengageKicker();
            if (indexMode != IndexMode.PASSTHROUGH)
                r.p.closeBottomGate();
            r.p.closeTopGate();
        }


        if (gamepad1.bWasPressed())
            shoot = !shoot;

        if (gamepad1.dpadUpWasPressed()) {
            if (r.a.equals(Alliance.BLUE)) {
                r.f.setPose(BaronPose.mirror(new Pose(7.5, 8.5, Math.toRadians(0))));
            } else {
                r.f.setPose(new Pose(7.5, 8.5, Math.toRadians(0)));
            }
        }

        if (gamepad1.dpadLeftWasPressed())
            manual = !manual;

        if (gamepad1.dpadRightWasPressed())
            field = !field;

        if (gamepad1.xWasPressed())
            r.t.resetTurret();

        if (gamepad1.left_bumper)
            speed = 0.5;
        else
            speed = 1.0;

        if (r.p.full() && !wasFull && !shooting) {
            if (indexMode == IndexMode.UNSORTED)
                r.p.moveTo(1);
            else if (indexMode == IndexMode.SORTED)
                r.p.optimal();
        }

        if (gamepad1.touchpadWasPressed())
            switchIndexing();

        if (gamepad1.rightStickButtonWasPressed())
            r.p.spin(1);

        if (gamepad1.leftStickButtonWasPressed())
            r.p.spin(-1);

        wasFull = r.p.full();
        wasDone = r.p.done.getElapsedTimeSeconds() > 0.25;

        telemetry.addData("LoopTime Hz", r.getLoopTimeHz());
        telemetry.addData("Slots", Arrays.toString(r.p.slots));
        telemetry.addData("Shooter Velocity", r.s.getVelocity());
        telemetry.addData("Turret Error", r.t.getError());
        telemetry.addData("Pose", r.f.getPose());
        telemetry.addData("Target", r.getShootTarget());
        telemetry.addData("Spindexer Shooting?", shooting);
        telemetry.addData("Done Timer", r.p.done.getElapsedTimeSeconds());
        telemetry.addData("Distance Sensor", r.p.dist);
        telemetry.addData("Index Mode", indexMode.toString());
//        multipleTelemetry.addData("Abs X", Math.abs(r.getShootTarget().getX()-r.f.getPose().getX()));
//        multipleTelemetry.addData("Abs Y", Math.abs(r.getShootTarget().getY()-r.f.getPose().getY()));
//        multipleTelemetry.addData("Shoot Target", shootTarget);
//        multipleTelemetry.addLine();
//        multipleTelemetry.addData("Follower Pose", r.f.getPose().toString());
//        multipleTelemetry.addData("Shooter Velocity", r.s.getVelocity());
//        multipleTelemetry.addData("Shooter Target", r.s.getTarget());
//        multipleTelemetry.addData("Shooter Distance", dist);
//        multipleTelemetry.addData("Turret Yaw", r.t.getYaw());
//        multipleTelemetry.addData("Turret Target", r.t.getTurretTarget());
//        multipleTelemetry.addData("Turret Ticks", r.t.getTurret());
//        multipleTelemetry.addData("Shooter On", shoot);
//        multipleTelemetry.addData("Automatic Flipping Running", autoFlipping);
//        multipleTelemetry.addData("Automatic Flipping Timer", autoFlipTimer.getElapsedTimeSeconds());
//        multipleTelemetry.addLine("Manual Flipper: " + manualFlip);
//        multipleTelemetry.addData("Distance from Target", dist);
//        multipleTelemetry.addData("Manual Shooter + Turret", manual);
//        multipleTelemetry.addData("Field Centric", field);
//        multipleTelemetry.addData("Hold Position", hold);
        telemetry.update();
    }


    @Override
    public void stop() {
        r.saveEnd();
    }

    public void switchIndexing() {
        if (indexMode == IndexMode.PASSTHROUGH) {
            indexMode = IndexMode.UNSORTED;
            r.p.closeTopGate();
            r.p.closeBottomGate();
            r.p.disableSort();
            r.p.enableAutoRotate();
        } else if (indexMode == IndexMode.UNSORTED) {
            indexMode = IndexMode.SORTED;
            r.p.closeTopGate();
            r.p.closeBottomGate();
            r.p.enableSort();
            r.p.enableAutoRotate();
        } else {
            indexMode = IndexMode.PASSTHROUGH;
            r.p.closeTopGate();
            r.p.openBottomGate();
            r.p.disableSort();
            r.p.disableAutoRotate();
            r.p.moveTo(2);
        }
    }
}