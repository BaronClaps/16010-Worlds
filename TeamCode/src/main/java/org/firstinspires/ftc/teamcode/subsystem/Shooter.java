package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.CachedMotor;
import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

@Config

public class Shooter {
    private final CachedMotor top, bottom;

    private double t = 0;
    public static double kS = 0.07, kV = 0.00036, kP = 0, useRaw = 100;

    private boolean activated = false;

    private static final double[] xs = {36, 60, 84, 108}; // abs(goal.getX() - current.getX())
    private static final double[] ys = {132, 108, 84}; // actual y value
    private static final double[][] closeVelocities = {
            {1700, 1700, 1700}, // x1 + y1, x1 + y2, x1 + y3
            {1700, 1700, 1700},
            {1700, 1700, 1700},
            {1700, 1700, 1700},
    };

    public static final Interpolation2D closeInterpolation = new BilinearInterpolation(xs, ys, closeVelocities);
    private double velocity;

    public Shooter(HardwareMap hardwareMap) {
        top = new CachedMotor(hardwareMap.get(DcMotorEx.class, "flywheel1"));
        bottom = new CachedMotor(hardwareMap.get(DcMotorEx.class, "flywheel2"));
        top.setDirection(DcMotorSimple.Direction.REVERSE);
        bottom.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setPower(double p) {
        top.setPower(p);
        bottom.setPower(p);
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public void setTarget(double velocity) {
        t = velocity;
    }

    public void periodic() {
        if (activated) {
            velocity = top.getVelocity();

            if (Math.abs(getTarget() - getVelocity()) > useRaw)
                setPower(Math.signum(getTarget() - getVelocity()));
            else
                setPower(Math.max(0,(kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS));
        }
    }

    public boolean atTarget() {
        return Math.abs((getTarget() - getVelocity())) < 40;
    }
    public boolean atTargetFar() {
        return Math.abs((getTarget() - getVelocity())) < 20;
    }

    public void forPose(Pose current, Pose target, boolean close) {
        double xDistance = Math.abs(target.getX() - current.getX());

        if (close)
            setTarget(closeInterpolation.interpolate(xDistance, current.getY()));
        else
            setTarget(2000); // make a far interpolation
    }

    public void forDistance(double distance, boolean close) {
        if (!close) {
            setTarget(getFar(distance));
        } else {
            setTarget(getClose(distance));
        }
    }

    public void forClose(double distance) {
        setTarget(getClose(distance));
    }

    public void forFar(double distance) {
        setTarget(getFar(distance));
    }

    public void close() {
        setTarget(1100);
        on();
    }

    public static double getClose(double x) {
        return 0.00259534 * Math.pow(x, 3)
                - 0.566792  * Math.pow(x, 2)
                + 45.23831  * x
                - 306.12169
                + 220;
    }

    public static double getFar(double x) {
        return 2.83784 * x + 947.97297 + 200;
    }

    public double getCurrent() {
        return top.getCurrent(CurrentUnit.AMPS) + bottom.getCurrent(CurrentUnit.AMPS);
    }
}

