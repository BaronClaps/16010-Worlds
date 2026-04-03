package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Turret {
    private final Servo left, right;
    private double position = 0.5;
    private static final double ppr = (.25 - .5) / (Math.PI / 2.0);

    public Turret(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "tl");
        right = hardwareMap.get(Servo.class, "tr");

        set(position);
    }

    private void set(double position) {
        left.setPosition(position);
        right.setPosition(position);
        this.position = position;
    }

    private double get() {
        return position;
    }

    public void manual(double increment) {
        set(get() + increment);
    }

    public double getYaw() {
        return normalizeAngle((get() - 0.5) / ppr);
    }

    public void setYaw(double radians) {
        radians = normalizeAngle(radians);
        set(0.5 + ppr * radians);
    }

    public void face(Pose targetPose, Pose robotPose) {
        double angleToTargetFromCenter = Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX());
        double robotAngleDiff = normalizeAngle(angleToTargetFromCenter - robotPose.getHeading());
        robotAngleDiff = MathFunctions.clamp(robotAngleDiff, -(Math.PI)/2, Math.PI/2);
        setYaw(robotAngleDiff);
    }

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI) angle -= Math.PI * 2D;
        return angle;
    }
}