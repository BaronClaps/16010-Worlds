package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

@TeleOp(group="Tests")
@Config
public class IntakeTest extends OpMode {
    Intake intake;
    Timer timer = new Timer();
    public boolean prev;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
    }

    public void start() {
        timer.resetTimer();
        prev = false;
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            intake.set(1);
        } else if (gamepad1.bWasPressed()) {
            intake.set(-1);
        } else if (gamepad1.yWasPressed()) {
            intake.set(0);
        }

        boolean curr = intake.isDetected();

        telemetry.addData("Distance", intake.getDistance());
        telemetry.addData("Detected", curr);

        // turn green only after detection has stayed true for 0.5s
        if (curr != prev) {
            timer.resetTimer();
        }

        if (curr && timer.getElapsedTimeSeconds() >= 0.5)
            intake.light.green();
        else
            intake.light.blue();

        prev = curr;
    }
}
