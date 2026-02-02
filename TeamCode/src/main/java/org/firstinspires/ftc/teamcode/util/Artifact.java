package org.firstinspires.ftc.teamcode.util;

public enum Artifact {
    // RGB calibration values are normalized (0..1) defaults — tune these for your sensor.
    PURPLE(0.35, 0.20, 0.85),
    GREEN(0.20, 0.80, 0.20),
    UNIDENTIFIED(0.0, 0.0, 0.0);

    public final double r;
    public final double g;
    public final double b;

    Artifact(double r, double g, double b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }
}