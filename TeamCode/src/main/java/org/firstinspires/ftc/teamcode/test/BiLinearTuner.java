package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Paths;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@TeleOp(group="Tests")
@Config
public class BiLinearTuner extends OpMode {
    Turret t;
    Shooter s;
    Follower f;
    public static double angle = 0, hood = .55, velocity = 1700, x = 72, y = 72;
    @Override
    public void init() {
//        t = hardwareMap.get(DcMotor.class, "t");
        t = new Turret(hardwareMap);
        t.resetTurret();
        s = new Shooter(hardwareMap);
        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(new Pose(31.3125, 144-11, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        s.on();
        s.setTarget(velocity);
        s.setHood(hood);
        t.automatic();
        t.on();
        t.setYaw(angle);
        f.holdPoint(new Pose(x,y, 0), false);
        f.update();
        s.periodic();
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
