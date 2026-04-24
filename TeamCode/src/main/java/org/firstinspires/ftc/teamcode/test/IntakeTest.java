package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;

@TeleOp(group="Tests")
@Config
public class IntakeTest extends OpMode {
    Intake intake;
    Transfer transfer;
    Timer timer = new Timer();
    public static double i, t;
    public boolean prev;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);

        intake.lower();
        transfer.close();
    }

    public void start() {
        timer.resetTimer();
        prev = false;
    }

    @Override
    public void loop() {
        intake.set(i);
        transfer.set(t);

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
