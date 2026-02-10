package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@TeleOp
@Config
public class ShootTest extends OpMode {
    Shooter s;
    Intake i;
    public static double ipower = 1, hood = .5, target = 1200;


    @Override
    public void init() {
        s = new Shooter(hardwareMap);
        i = new Intake(hardwareMap);
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
        s.setTarget(1700);
    }

    @Override
    public void loop() {
        i.set(ipower);
        s.setHood(hood);
        s.periodic();
        s.setTarget(target);

        telemetry.addData("Shooter V", s.getVelocity());
        telemetry.addData("Shooter T", s.getTarget());
        telemetry.update();
    }
}
