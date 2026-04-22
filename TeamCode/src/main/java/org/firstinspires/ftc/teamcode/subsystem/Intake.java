package org.firstinspires.ftc.teamcode.subsystem;

import android.health.connect.datatypes.ExerciseRoute;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.CachedMotor;
import org.firstinspires.ftc.teamcode.util.RGBLight;

@Config
public class Intake {
    private final CachedMotor intake;
    private final Servo pivot;
    private final RevColorSensorV3 color;
    public final RGBLight light;
    public static double off = 0;
    public static double idle = 0.5;
    public static double in = 1;
    public static double out = -1;
    public static double up = 0;
    public static double down = 0.33;
    public static double lowerThreshold = 0, upperThreshold = 2;

    public Intake(HardwareMap hardwareMap) {
        intake = new CachedMotor(hardwareMap.get(DcMotorEx.class, "intake"));
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot = hardwareMap.get(Servo.class, "intakePivot");
        color = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        light = new RGBLight(hardwareMap.get(Servo.class, "light"));
        set(0);
    }

    public void set(double power) {
        intake.setPower(power);
    }

    public void in() {
        set(in);
    }
    public void out() {
        set(out);
    }
    public void off() {
        set(off);
    }
    public void idle() {
        set(idle);
    }
    public void raise() {
        pivot.setPosition(up);
    }
    public void lower() {
        pivot.setPosition(down);
    }
    public double getDistance() {
        return color.getDistance(DistanceUnit.INCH);
    }

    public boolean isDetected(double distance) {
        return (distance > lowerThreshold && distance < upperThreshold);
    }

    public boolean isDetected() {
        double distance = getDistance();
        return isDetected(distance);
    }

    public CommandBuilder offCommand() {
        return Commands.instant(this::off);
    }
    public CommandBuilder inCommand() {
        return Commands.instant(this::in);
    }
    public CommandBuilder outCommand() {
        return Commands.instant(this::out);
    }
    public CommandBuilder idleCommand() {
        return Commands.instant(this::idle);
    }
    public CommandBuilder raiseCommand() {
        return Commands.instant(this::raise);
    }
    public CommandBuilder lowerCommand() {
        return Commands.instant(this::lower);
    }
}