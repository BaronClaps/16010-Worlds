package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.InterpLUT;
import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Config

public class Shooter {
    private DcMotorEx l, r;
    private Servo h;

    private double t = 0;
    public static double kS = 0.08, kV = 0.00039, kP = 0.01, useRaw = 250;
    // .22 to .78

    private boolean activated = true;

    private static final double[] xs = {44, 72, 100};
    private static final double[] ys = {10, 38, 66};
    private static final double[][] closeVelocities = {
            {1200, 1200, 1275},
            {1275, 1275, 1350},
            {1325, 1360, 1400}
    };

    private static final List<Double> distances =
            List.of(35.0, 40.0, 45.0, 50.0, 55.0, 80.0);

    private static final List<Double> velocities =
            List.of(1700.0, 1750.0, 1800.0, 1850.0, 1900.0, 1800.0);

    private static final List<Double> hoods =
            List.of(.5, .55, .6, .65, .7, .55);

    public InterpLUT shooterILUT = new InterpLUT(distances, velocities);
    public InterpLUT hoodILUT = new InterpLUT(distances, hoods);
    public static final Interpolation2D closeInterpolation = new BilinearInterpolation(xs, ys, closeVelocities);
    public Shooter(HardwareMap hardwareMap) {
        h = hardwareMap.get(Servo.class, "h");
        l = hardwareMap.get(DcMotorEx.class, "sl");
        r = hardwareMap.get(DcMotorEx.class, "sr");
        l.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return -l.getVelocity();
    }

    public void setPower(double p) {
        l.setPower(p);
        r.setPower(p);
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public void shooterToggle() {
        activated = !activated;
        if (!activated)
            setPower(0);
    }

    public void setTarget(double velocity) {
        t = velocity;
    }

    public void periodic() {
        if (activated)
            if (Math.abs(getTarget() - getVelocity()) > useRaw)
                setPower(Math.signum(getTarget() - getVelocity()));
                else
                setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);
    }

    public boolean atTarget() {
        return Math.abs((getTarget()- getVelocity())) < 50;
    }

    public void forPose(Pose current, Pose target, boolean close) {
        double xDistance = Math.abs(target.getX()-current.getX());
        double yDistance = Math.abs(target.getY()-current.getY());

        if (close)
            setTarget(closeInterpolation.interpolate(xDistance, yDistance) + 500);
        else
            setTarget(2000);
    }

    public void forDistance(double distance) {
        setHood(hoodILUT.get(distance));
        setTarget(shooterILUT.get(distance));
    }

    public double timeInAir() {
        return 1.0;
    }

    public void setHood(double p) { h.setPosition(p);}

}

