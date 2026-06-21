package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Transfer;

@TeleOp(group="Tests")
@Config
public class TransferGateTest extends OpMode {
    Transfer t;
    @Override
    public void init() {
        t = new Transfer(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed())
            t.open();

        if (gamepad1.bWasPressed())
            t.close();

    }
}
