package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.Robot;

public final class SOTM {
    private static final int ITERATIONS = 10;

    public static double getTOF(double distance) {
        return (0.00486758*distance)+0.542966;
    }

    public static Pose calculateVirtualRobot(Pose robotPose, Vector robotVelocity, double dist) {
        if (robotVelocity.getMagnitude() < 1) return robotPose;
        Vector virtualPose = robotPose.getAsVector();
        Vector robotVectorPose = robotPose.getAsVector();
        for (int i = 0; i < ITERATIONS; i++) {
            double airTime = getTOF(dist);
            virtualPose = robotVectorPose.plus(robotVelocity.times(airTime));
        }
        return new Pose(virtualPose.getXComponent(), virtualPose.getYComponent(), robotPose.getHeading());
    }
}