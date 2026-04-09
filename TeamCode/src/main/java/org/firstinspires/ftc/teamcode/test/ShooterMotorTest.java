package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group="Tests")
@Config
public class ShooterMotorTest extends OpMode {
    DcMotor l,r;
    @Override
    public void init() {
        l = hardwareMap.get(DcMotor.class, "flywheel1");
        r = hardwareMap.get(DcMotor.class, "flywheel2");
    }

    @Override
    public void loop() {
        l.setPower(gamepad1.left_stick_y);
        r.setPower(gamepad1.right_stick_y);
    }
}
