package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@TeleOp(group="Tests")
@Config
public class TurretTest extends OpMode {
    Turret turret;
    public static double yaw = 0, position = 0.5;
    public static boolean useYaw = false;
    @Override
    public void init() {
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {
        if (useYaw)
            turret.setYaw(yaw);
        else
            turret.set(position);
    }
}
