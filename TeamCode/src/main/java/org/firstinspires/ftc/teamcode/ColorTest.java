package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class ColorTest extends OpMode {
    Servo l, r;
    RevColorSensorV3 color;
    public static double p = 0.15;
    @Override
    public void init() {
       // l = hardwareMap.get(Servo.class, "spl");
        r = hardwareMap.get(Servo.class, "spr");
        color = hardwareMap.get(RevColorSensorV3.class, "color");
    }

    @Override
    public void loop() {
        //l.setPosition(p);
        r.setPosition(p);

        telemetry.addData("dist", color.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
}
