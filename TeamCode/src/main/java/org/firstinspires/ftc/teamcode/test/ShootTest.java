package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;

@TeleOp(group="Tests")
@Config
public class ShootTest extends OpMode {
    Shooter s;
    Intake i;
    Transfer t;
    public static double ipower = 0, tpower = 0, target = 1650;


    @Override
    public void init() {
        s = new Shooter(hardwareMap);
        i = new Intake(hardwareMap);
        t = new Transfer(hardwareMap);
        s.off();
    }

    @Override
    public void init_loop() {
        s.periodic();

        if (gamepad1.a)
            s.setPower(1);

        telemetry.addData("Shooter V", s.getVelocity());
        telemetry.update();
    }

    public void start() {
        s.on();
        s.setTarget(1650);
    }

    @Override
    public void loop() {
        i.set(ipower);
        t.set(tpower);
        s.periodic();
        s.setTarget(target);

        telemetry.addData("Shooter V", s.getVelocity());
        telemetry.addData("Shooter T", s.getTarget());
        telemetry.update();
    }
}
