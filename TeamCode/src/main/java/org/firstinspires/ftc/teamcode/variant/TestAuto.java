package org.firstinspires.ftc.teamcode.variant;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Test Auto", group="Autonomous")
public class TestAuto extends OpMode {
    Timer autoTimer = new Timer();
    public Pose start = new Pose(32, 130.5, Math.toRadians(90));
    public Pose score = new Pose(55, 89, Math.toRadians(200));

    public Pose spike1 = new Pose(17.5, 82, Math.toRadians(180));
    public Pose spike1Control1 = new Pose(48, 76);
    public Pose spike1Control2 = spike1.withX(39.5);

    public Pose spike2 = new Pose(8, 57, Math.toRadians(180));
    public Pose spike2Control1 = new Pose(45, 69);
    public Pose spike2Control2 = spike2.withX(50);

    public Pose spike3 = new Pose(10, 36, Math.toRadians(180));
    public Pose spike3Control1 = new Pose(45, 56);
    public Pose spike3Control2 = spike3.withX(65);

    public Pose gateHit = new Pose(15, 70, Math.toRadians(180));
    public Pose gateHitControl = gateHit.withX(32);
    public Pose gateIntake = new Pose(9, 60, Math.toRadians(140));
    public Pose gateControl1 = new Pose(28, 60, Math.toRadians(140));
    public Pose gateControl2 = new Pose(23.25, 49);

    public Pose cornerControl = new Pose(-5, 30);
    public Pose corner = new Pose(6.5, 11, Math.toRadians(270));

    public Pose park = new Pose(48, 120, Math.toRadians(180));
    public PathChain preload;
    Robot robot;
    private int step = -1;

    public void init() {
        robot.follower.setStartingPose(start);
        robot.follower.update();

        preload = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        start,
                        score
                    )
                )
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading(), .5)
                .setBrakingStrength(2)
                .build();
    }

    public void loop() {
        robot.shooter.on();
        robot.shooter.forDistance(robot.getShootTarget().distanceFrom(score), score.getY() > 48);
        robot.intake.off();
        robot.transfer.off();
        robot.transfer.open();
        robot.turret.face(robot.getShootTarget(), score);
        robot.follower.update();
        robot.shooter.periodic();
        switch(step) {
            case 0:
                robot.follower.followPath(preload);
                setPathState(1);
                break;
            case 1:
                if(autoTimer.getElapsedTimeSeconds() > 1.85) {
                    robot.shoot(score);
                }
                break;
        }
    }

    public void start() {
        step = 0;
    }

    public void setPathState(int num) {
        step = num;
        autoTimer.resetTimer();
    }
}
