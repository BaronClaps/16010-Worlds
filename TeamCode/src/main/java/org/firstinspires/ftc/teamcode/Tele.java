package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Robot.defaultPose;
import static org.firstinspires.ftc.teamcode.subsystem.Spindexer.timeToShoot;

@Config
public class Tele extends CommandOpMode {

    Robot r;
    final Alliance a;

    public boolean shoot = false, manual = false, field = true, wasFull = false, shooting = false, all = false;
    public double intakeOn = 0, speed = 1;
    private final Timer shootTimer = new Timer();
    public static double shootTarget = 1700, hoodTarget = 0.5;

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
                double dist = r.getShootTarget().distanceFrom(r.f.getPose());
//                boolean close = r.f.getPose().getY() > 48;
                r.s.forDistance(dist);
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

        if (gamepad1.aWasPressed() && shoot) {
            shootTimer.resetTimer();
            r.i.spinIn();
            intakeOn = 1;
            r.p.engageKicker();
            r.p.openBottomGate();
            r.p.all();
            shooting = true;
            all = true;
        }

        if (gamepad1.yWasPressed()) {
            shootTimer.resetTimer();
            r.i.spinIn();
            intakeOn = 1;
            r.p.engageKicker();
            r.p.openBottomGate();
            r.p.all();
            shooting = true;
        }

        if (shootTimer.getElapsedTimeSeconds() > timeToShoot + 0.3 && shooting) {
            shooting = false;
            intakeOn = 1;
            r.i.spinIn();
            r.p.disengageKicker();
            r.p.closeBottomGate();
        }


        if (gamepad1.bWasPressed())
            shoot = !shoot;

        if (gamepad1.dpadUpWasPressed()) {
            if (r.a.equals(Alliance.BLUE)) {
                r.f.setPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
            } else {
                r.f.setPose(new Pose(8, 6.25, Math.toRadians(0)));
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
            r.p.optimal();
            r.p.openBottomGate();
            r.p.disengageKicker();
        }

        if (gamepad1.rightStickButtonWasPressed())
            r.p.spin(1);

        if (gamepad1.leftStickButtonWasPressed())
            r.p.spin(-1);

        wasFull = r.p.full();

        telemetry.addData("LoopTime Hz", r.getLoopTimeHz());
        telemetry.addData("Slots", Arrays.toString(r.p.slots));
        telemetry.addData("Shooter Velocity", r.s.getVelocity());
        telemetry.addData("Turret Error", r.t.getError());
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
}