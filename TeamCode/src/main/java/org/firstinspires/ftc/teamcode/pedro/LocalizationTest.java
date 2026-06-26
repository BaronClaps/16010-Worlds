package org.firstinspires.ftc.teamcode.pedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp
public class LocalizationTest extends OpMode {
    private Follower follower;
    private MultipleTelemetry multipleTelemetry;
    public double loops = 0, lastLoop = 0, loopTime = 0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));

        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        loops++;

        if (loops > 10) {
            double now = System.currentTimeMillis();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        Mecanum m = (Mecanum) follower.drivetrain;
        follower.update();
        multipleTelemetry.addData("Loop Time Hz", 1000/loopTime);
        multipleTelemetry.addData("Mode", follower.mode());
        multipleTelemetry.addData("Idle?", follower.idle());
        multipleTelemetry.addData("Pose", follower.pose());
        multipleTelemetry.addData("Manual Powers", follower.manualPowers);
        multipleTelemetry.addData("Manual Output", Arrays.toString(Arrays.stream(m.wheelPowers).toArray()));
        multipleTelemetry.update();
    }
}
