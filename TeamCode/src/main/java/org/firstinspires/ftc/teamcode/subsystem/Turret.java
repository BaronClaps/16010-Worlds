package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CachedMotor;

@Config
public class Turret {
    private double error = 0;
    public double power = 0;
    private double manualPower = 0, currentPosition;
    public static double rpt = 0.00866048974;
    public static double kShift = 0.25; //.12 for linear, .25 for angular // Inches of shift per inch away from wall //
    public static double maxShift = 12;
    public static boolean tuning = false;

    public final CachedMotor m;
    private final PIDFController p, s; // pidf controller for turret
    private double t = 0;
    public static double pidfSwitch = 15; // target for turret
    public static double kp = 1, kf = 0.05, kd = 0.005, sp = 0.2, sf = 0.03, sd = 0.001;

    public static boolean on = false, manual = false;

    public Turret(HardwareMap hardwareMap) {
        m = new CachedMotor(hardwareMap.get(DcMotorEx.class, "t"));
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
        return currentPosition;
    }

    public void periodic() {
        if (on) {
            currentPosition = m.getCurrentPosition();

            if (manual) {
                m.setPower(manualPower);
                return;
            }

            if (tuning) {
                p.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
                s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));
            }

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
        double angleToTargetFromCenter = Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX());
        double robotAngleDiff = normalizeAngle(angleToTargetFromCenter - robotPose.getHeading());
        robotAngleDiff = MathFunctions.clamp(robotAngleDiff, -(Math.PI)/2, Math.toRadians(135));
        setYaw(robotAngleDiff);
    }

    public void face(Pose targetPose, Pose robotPose, Alliance alliance) {
        double tx = targetPose.getX();
        double ty = targetPose.getY();

        double dx = tx - robotPose.getX();
        double dy = ty - robotPose.getY();

        double diag = (alliance == Alliance.RED) ? Math.PI / 4.0 : 0.75 * Math.PI;
        double offset = diag - Math.atan2(dy, dx);
        double shift = Math.min(maxShift, Math.abs(Math.toDegrees(offset)) * kShift);

        if (alliance == Alliance.RED) {
            if (offset < 0) tx -= shift;
            else ty -= shift;
        } else {
            if (offset > 0) tx += shift;
            else ty -= shift;
        }

        face(new Pose(tx, ty), robotPose);
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
        return Math.abs(getError()) < 12;
    }

    public void setPowerZero() {
        m.setPower(0);
    }
}