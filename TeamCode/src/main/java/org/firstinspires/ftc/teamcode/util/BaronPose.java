package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

public class BaronPose {
    public static Pose mirror(Pose k) {
        k = k.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        return new Pose(141.5 - k.getX(), k.getY(), MathFunctions.normalizeAngle(Math.PI - k.getHeading()), PedroCoordinates.INSTANCE);
    }
}
