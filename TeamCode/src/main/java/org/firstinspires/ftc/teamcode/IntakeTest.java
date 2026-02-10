package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Config
public class IntakeTest extends OpMode {
    DcMotor intake;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "i");
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            intake.setPower(1);
        } else if (gamepad1.bWasPressed()) {
            intake.setPower(-1);
        } else if (gamepad1.yWasPressed()) {
            intake.setPower(0);
        }
    }
}
