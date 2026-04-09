package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@TeleOp(group="Tests")
@Config
public class ShootTest extends OpMode {
    Shooter s;
    Intake i;
    Transfer t;
    Turret turret;
    MultipleTelemetry telemetryM;
    public static double ipower = 0, tpower = 0, target = 1025, turretYaw = 0;


    @Override
    public void init() {
        s = new Shooter(hardwareMap);
        i = new Intake(hardwareMap);
        t = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        s.off();

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        s.periodic();

        if (gamepad1.a)
            s.setPower(1);

        telemetryM.addData("Shooter V", s.getVelocity());
        telemetryM.update();
    }

    public void start() {
        s.on();
        s.setTarget(1025);
    }

    @Override
    public void loop() {
        i.set(ipower);
        t.set(tpower);
        turret.setYaw(turretYaw);
        s.periodic();
        s.setTarget(target);

        telemetryM.addData("Shooter V", s.getVelocity());
        telemetryM.addData("Shooter T", s.getTarget());
        telemetryM.update();
    }
}
