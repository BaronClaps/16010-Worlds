package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.866)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.25, 0.0788,.00191035))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5, 0, .01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.015, 0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.015,0,0.003,0))
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .useBrakeModeInTeleOp(true)
            .xVelocity(81.175)
            .yVelocity(66.8431)
            .leftFrontMotorName("lf")
            .rightFrontMotorName("rf")
            .leftRearMotorName("lb")
            .rightRearMotorName("rb")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("p")
            .forwardPodY(4.1871)
            .strafePodX(-6.433)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 50, 1.5, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }

    public static Follower createFollower(HardwareMap hardwareMap, Localizer localizer) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .setLocalizer(localizer)
                .build();
    }
}