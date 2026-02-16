package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group="Tests")
@Config
public class TransferTest extends OpMode {
    DcMotor l,r,i;
    Servo k, tg, bg;
    public static double power, ipower = 0, kp= 1, tgp = 0.575, bgp = 0.5;


    @Override
    public void init() {
        i = hardwareMap.get(DcMotor.class, "i");
        l = hardwareMap.get(DcMotor.class, "sl");
        r = hardwareMap.get(DcMotor.class, "sr");
        k = hardwareMap.get(Servo.class, "k");
        tg = hardwareMap.get(Servo.class, "tg");
        bg = hardwareMap.get(Servo.class, "bg");

        l.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        l.setPower(power);
        r.setPower(power);
        i.setPower(ipower);
        k.setPosition(kp);
        tg.setPosition(tgp);
        bg.setPosition(bgp);

    }
}
