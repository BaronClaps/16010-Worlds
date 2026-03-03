package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Turret {
    private double error = 0;
    public static double power = 0;
    private double manualPower = 0;
    public static double rpt = 0.00866048974, turretOffset = 3.3111811;

    public final DcMotorEx m;
    private final PIDFController p, s; // pidf controller for turret
    private double t = 0;
    public static double pidfSwitch = 15; // target for turret
    public static double kp = 1, kf = 0.05, kd = 0.005, sp = 0.2, sf = 0.03, sd = 0.001;

    public static boolean on = true, manual = false;

    public Turret(HardwareMap hardwareMap) {
        m = hardwareMap.get(DcMotorEx.class, "t");
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setPowerZero();
        t = 0;
        power = 0;

        p = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
    }

    private void setTurretTarget(double ticks) {
        t = ticks;
    }

    public double getTurretTarget() {
        return t;
    }

    public double getTurret() {
        return m.getCurrentPosition();
    }

    public void periodic() {
        if (on) {
            if (manual) {
                m.setPower(manualPower);
                return;
            }

            p.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
            s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

            error = getTurretTarget() - getTurret();
            double errorInRadians = error * rpt;

            if (Math.abs(error) > pidfSwitch) {
                p.updateError(errorInRadians);
                p.updateFeedForwardInput(Math.signum(errorInRadians));
                power = p.run();
                m.setPower(power);
            } else {
                s.updateError(errorInRadians);
                s.updateFeedForwardInput(Math.signum(errorInRadians));
                power = s.run();
                m.setPower(power);
            }
        } else {
            m.setPower(0);
        }
    }

    public void manual(double power) {
        manual = true;
        manualPower = power;
    }

    public void automatic() {
        manual = false;
    }

    public void on() {
        on = true;
    }

    public void off() {
        on = false;
    }

    public double getYaw() {
        return normalizeAngle(getTurret() * rpt);
    }

    public void setYaw(double radians) {
        radians = normalizeAngle(radians);
        setTurretTarget(radians/rpt);
    }

    public void addYaw(double radians) {
        setYaw(getYaw() + radians);
    }

    public void face(Pose targetPose, Pose robotPose) {

//        double heading = robotPose.getHeading();
//
//        double offsetXRobot = turretOffset;
//        double offsetYRobot = 0;
//
//        double cos = Math.cos(heading);
//        double sin = Math.sin(heading);
//
//        double offsetXField = offsetXRobot * cos - offsetYRobot * sin;
//        double offsetYField = offsetXRobot * sin + offsetYRobot * cos;
//
//        double turretX = robotPose.getX() + offsetXField;
//        double turretY = robotPose.getY() + offsetYField;
//
//        double angleToTarget =
//                Math.atan2(targetPose.getY() - turretY,
//                        targetPose.getX() - turretX);
//
//        double robotAngleDiff =
//                normalizeAngle(angleToTarget - heading);
//
//        robotAngleDiff =
//                MathFunctions.clamp(robotAngleDiff,
//                        -Math.PI/2,
//                        Math.PI/2);
//
//        setYaw(robotAngleDiff);
        double angleToTargetFromCenter = Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX());
        double robotAngleDiff = normalizeAngle(angleToTargetFromCenter - robotPose.getHeading());
        robotAngleDiff = MathFunctions.clamp(robotAngleDiff, -(Math.PI)/2, Math.toRadians(135));
        setYaw(robotAngleDiff);
    }

    public void resetTurret() {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTurretTarget(0);
    }

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI) angle -= Math.PI * 2D;
        return angle;
    }

    public double getError() {
        return error;
    }

    public boolean isReady() {
        return Math.abs(getError()) < 20;
    }

    public void setPowerZero() {
        m.setPower(0);
    }
}