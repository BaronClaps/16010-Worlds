package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.CachedMotor;

@Config
public class Transfer {
    private final CachedMotor transfer;
    private final Servo gate;
    public static double off = 0;
    public static double idle = 0.5;
    public static double in = 1;
    public static double out = -1;
    public static double open = 0.7;
    public static double closed = 0;

    public Transfer(HardwareMap hardwareMap) {
        transfer = new CachedMotor(hardwareMap.get(DcMotorEx.class, "transfer"));
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        gate = hardwareMap.get(Servo.class, "shooterGate");
        set(0);
    }

    public void set(double power) {
        transfer.setPower(power);
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
    public void open() {
        gate.setPosition(open);
    }
    public void close() {
        gate.setPosition(closed);
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
    public CommandBuilder openCommand() {
        return Commands.instant(this::open);
    }
    public CommandBuilder closeCommand() {
        return Commands.instant(this::close);
    }
}