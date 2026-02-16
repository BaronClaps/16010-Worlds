package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.util.Pattern;

@TeleOp(group="Tests")
@Config
public class SpindexTest extends OpMode {
    Spindexer s;
    Intake i;

    public static Pattern pattern = Pattern.GPP;

    @Override
    public void init() {
        s = new Spindexer(hardwareMap);
        i = new Intake(hardwareMap);

        s.setPattern(pattern);
        s.moveTo(2);
    }

    @Override
    public void loop() {
        // Automatically handles sensor reading, empty-slot positioning,
        // and calling optimal() once the slots are full.
        s.periodic();

        // Manual shoot trigger - periodic() has already set up the position/direction
        if (gamepad1.aWasPressed()) {
            s.all();
        }

        // Manual index adjustments
        if (gamepad1.bWasPressed()) {
            s.spin(1);
        }

        if (gamepad1.xWasPressed()) {
            s.spin(-1);
        }

        // Logic State Overrides
        if (gamepad1.dpadDownWasPressed()) {
            s.disableSort();
            s.disableAutoRotate();
        }

        if (gamepad1.dpadLeftWasPressed()) {
            s.enableSort();
        }

        if (gamepad1.dpadRightWasPressed()) {
            s.enableAutoRotate();
            s.disableSort();
        }

        // Intake Controls
        if (gamepad1.right_bumper) {
            i.spinIn();
        } else if (gamepad1.left_bumper) {
            i.spinOff();
        }

        // Telemetry for debugging the "Lazy Optimal" logic
        telemetry.addData("Pattern", pattern);
        telemetry.addData("Full?", s.full());
        telemetry.addData("Current Index", s.currentIndex);
        telemetry.addData("Dist (in)", "%.2f", s.sensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Detected Color", s.getSimpleColor());
        telemetry.update();
    }
}