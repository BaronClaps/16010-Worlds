package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@TeleOp
@Config
public class TurretTest extends OpMode {
//    DcMotor t;
    Turret t;
    public static double angle = 0;
    @Override
    public void init() {
//        t = hardwareMap.get(DcMotor.class, "t");
        t = new Turret(hardwareMap);
        t.resetTurret();
    }

    @Override
    public void loop() {
        t.automatic();
        t.on();
        t.setYaw(angle);
        telemetry.addData("Turret Angle", t.getYaw());
        telemetry.addData("Error", t.getError());
        telemetry.addData("Turret Target", t.getTurretTarget());
        telemetry.addData("Current Position Ticks", t.getTurret());
        telemetry.addData("Power", t.power);
        telemetry.addLine("wow");
        telemetry.update();
        t.periodic();
//        if (gamepad1.a) {
//            t.setPower(1);
//        } else if (gamepad1.b) {
//            t.setPower(-1);
//        } else {
//            t.setPower(0);
//        }
    }
}
