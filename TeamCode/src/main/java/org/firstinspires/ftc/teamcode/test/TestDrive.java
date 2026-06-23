package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@TeleOp(group="Tests")
public class TestDrive extends OpMode {
    Shooter shooter;
    Intake intake;
    Transfer transfer;
    Turret turret;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private int vel = 0;
    private double intakePower = 0;
    private double drivetrainCurrent;

    public void init() {
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);

        frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        backLeft = hardwareMap.get(DcMotorEx.class, "lb");
        backRight = hardwareMap.get(DcMotorEx.class, "rb");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void start() {
        transfer.close();
        turret.set(0.5);
        shooter.setTarget(vel);
        shooter.on();
    }

    public void loop() {
        shooter.setTarget(vel);
        shooter.on();
        intake.set(intakePower);
        transfer.set(intakePower);

        if(gamepad1.right_trigger > 0.1) {
            vel = 1000;
        } else if(gamepad1.left_trigger > 0.1) {
            vel = 0;
        }

        if(gamepad1.rightBumperWasPressed()) {
            intakePower = 1;
        } else if(gamepad1.leftBumperWasPressed()) {
            intakePower = 0;
        }

        if(gamepad1.yWasPressed()) {
            transfer.open();
        } else if(gamepad1.bWasPressed()) {
            transfer.close();
        }

        if(gamepad1.dpadUpWasPressed()) {
            turret.set(0.5);
        } else if(gamepad1.dpadDownWasPressed()) {
            turret.set(0);
        }

        double max;

        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        drivetrainCurrent = (frontLeft.getCurrent(CurrentUnit.AMPS) + frontRight.getCurrent(CurrentUnit.AMPS) + backLeft.getCurrent(CurrentUnit.AMPS) + backRight.getCurrent(CurrentUnit.AMPS));
        double sAmps = shooter.getCurrent();
        double tAmps = transfer.getCurrent();
        double iAmps = intake.getCurrent();
        double totalAmps = drivetrainCurrent + sAmps + tAmps + iAmps;

        shooter.periodic();

        telemetry.addData("Drivetrain Amps: ", drivetrainCurrent);
        telemetry.addData("Shooter Amps: ", sAmps);
        telemetry.addData("Transfer Amps: ", tAmps);
        telemetry.addData("Intake Amps: ", iAmps);
        telemetry.addData("Total Amps: ", totalAmps);
    }
}