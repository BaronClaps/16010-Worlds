package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.CachedMotor;

@Config
public class Intake {
    private final CachedMotor intake;
    private final Servo pivot;
    public static double off = 0;
    public static double idle = 0.5;
    public static double in = 1;
    public static double out = -1;
    public static double up = 0.35;
    public static double down = 0;

    public Intake(HardwareMap hardwareMap) {
        intake = new CachedMotor(hardwareMap.get(DcMotorEx.class, "intake"));
        pivot = hardwareMap.get(Servo.class, "intakePivot");
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