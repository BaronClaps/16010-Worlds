package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Vector2D;

public final class SOTM {
    private static final int ITERATIONS = 10;

    public static double getTOF(double distance) {
        return (0.00486758*distance)+0.542966;
    }

    public static Pose calculateVirtualRobot(Pose robotPose, Vector2D robotVelocity, double dist) {
        if (robotVelocity.magnitude() < 1) return robotPose;
        Vector2D virtualPose = robotPose.toVector2D();
        Vector2D robotVectorPose = robotPose.toVector2D();
        for (int i = 0; i < ITERATIONS; i++) {
            double airTime = getTOF(dist);
            virtualPose = robotVectorPose.plus(robotVelocity.times(airTime));
        }
        return new Pose(virtualPose.x(), virtualPose.y(), robotPose.heading());
    }
}