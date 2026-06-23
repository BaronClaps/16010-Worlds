package org.firstinspires.ftc.teamcode.pedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class HoldTest extends OpMode {
    public static double DISTANCE = 24;

    private Follower follower;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));

        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        follower.hold(new Pose(72, 72, 0));
    }

    @Override
    public void loop() {
        follower.update();
        multipleTelemetry.addData("Mode", follower.mode());
        multipleTelemetry.addData("Idle?", follower.idle());
        multipleTelemetry.addData("Pose", follower.pose());
        multipleTelemetry.update();
    }
}
