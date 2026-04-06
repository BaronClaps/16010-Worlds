package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.CachedMotor;
import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

@Config

public class Shooter {
    private final CachedMotor top, bottom;

    private double t = 0;
    public static double kS = 0.08, kV = 0.00039, kP = 0.01, useRaw = 50;

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
        bottom.setDirection(DcMotorSimple.Direction.REVERSE);
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
                setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);
        }
    }

    public boolean atTarget() {
        return Math.abs((getTarget() - getVelocity())) < 50;
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
            setTarget(2000);
        } else {
            if (distance > 90) {
                setTarget(1800);
            }
            setTarget(1650);
        }
    }

    public void close() {
        setTarget(1200);
        on();
    }
}

