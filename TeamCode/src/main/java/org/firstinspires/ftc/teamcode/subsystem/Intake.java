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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.CachedMotor;
import org.firstinspires.ftc.teamcode.util.RGBLight;

@Config
public class Intake {
    private final CachedMotor intake;
    private final RevColorSensorV3 color;
    public static double off = -.0001;
    public static double in = 1;
    public static double out = -1;
    public static double lowerThreshold = 0, upperThreshold = 2;

    public Intake(HardwareMap hardwareMap) {
        intake = new CachedMotor(hardwareMap.get(DcMotorEx.class, "intake"));
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        color = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
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
    public double getDistance() {
        return color.getDistance(DistanceUnit.INCH);
    }
    public double getCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }
    public boolean isDetected() {
        double distance = getDistance();
        return isDetected(distance);
    }
    public boolean isDetected(double distance) {
        return (distance > lowerThreshold && distance < upperThreshold);
    }
    public CommandBuilder setCommand(double p) {
        return Commands.instant(() -> set(p));
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
}