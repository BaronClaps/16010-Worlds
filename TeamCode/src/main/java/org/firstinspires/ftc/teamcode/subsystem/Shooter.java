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
            List.of(35.0, 40.0, 45.0, 50.0, 55.0, 80.0, 120.0);

    private static final List<Double> velocities =
            List.of(1700.0, 1750.0, 1800.0, 1850.0, 1900.0, 1800.0, 2000.0);

    private static final List<Double> hoods =
            List.of(.5, .55, .6, .65, .7, .55, .65);

    public InterpLUT shooterILUT;
    public InterpLUT hoodILUT;
    public static final Interpolation2D closeInterpolation = new BilinearInterpolation(xs, ys, closeVelocities);

    public Shooter(HardwareMap hardwareMap) {
        h = hardwareMap.get(Servo.class, "h");
        l = hardwareMap.get(DcMotorEx.class, "sl");
        r = hardwareMap.get(DcMotorEx.class, "sr");
        l.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterILUT = new InterpLUT(distances, velocities);
        hoodILUT = new InterpLUT(distances, hoods);
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
        return Math.abs((getTarget() - getVelocity())) < 50;
    }

    public void forPose(Pose current, Pose target, boolean close) {
        double xDistance = Math.abs(target.getX() - current.getX());
        double yDistance = Math.abs(target.getY() - current.getY());

        if (close)
            setTarget(closeInterpolation.interpolate(xDistance, yDistance) + 500);
        else
            setTarget(2000);
    }

    public void forDistance(double distance, boolean close) {
        if (!close) {
            setTarget(2000);
            setHood(.6);
        } else {
//            if (distance < 120 && distance > 35) {
//                setHood(hoodILUT.get(distance));
//                setTarget(shooterILUT.get(distance));
//            } else {
            //setTarget(953.09995 * Math.pow(distance, 0.166662));
            setTarget((0.00255451 * Math.pow(distance, 3)) - (0.557218 * Math.pow(distance, 2)) + (41.91515 * distance) + 805.00963);
            double hood = (-0.000327755 * Math.pow(distance, 2)) + (0.0391562 * distance) - 0.482427;

            if (hood >= .77)
                hood = .77;

            if (hood <= .22)
                hood = .22;

            setHood(hood);
//            }

        }
    }

    public double timeInAir() {
        return 1.0;
    }

    public void setHood(double p) {
        h.setPosition(p);
    }

    public void close() {
        setTarget(1750);
        setHood(0.5);
        on();
    }

}

