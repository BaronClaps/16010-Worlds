package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.CachedMotor;
import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

import java.util.List;

@Config

public class Shooter {
    private CachedMotor l, r;
    private Servo h;

    private double t = 0;
    public static double kS = 0.08, kV = 0.00039, kP = 0.01, useRaw = 250;
    // .22 to .78

    private boolean activated = false;

    private static final double[] xs = {36, 60, 84, 108}; // abs(goal.getX() - current.getX())
    private static final double[] ys = {132, 108, 84}; // actual y value
    private static final double[][] closeVelocities = {
            {1700, 1700, 1700}, // x1 + y1, x1 + y2, x1 + y3
            {1700, 1700, 1700},
            {1700, 1700, 1700},
            {1700, 1700, 1700},
    };

    private static final double[][] closeHood = {
            {0.55, 0.55, 0.55}, // x1 + y1, x1 + y2, x1 + y3
            {0.55, 0.55, 0.55}, // servo positions
            {0.55, 0.55, 0.55},
            {0.55, 0.55, 0.55},
    };

    private static final List<Double> distances =
            List.of(35.0, 40.0, 45.0, 50.0, 55.0, 80.0, 120.0);

    private static final List<Double> velocities =
            List.of(1700.0, 1750.0, 1800.0, 1850.0, 1900.0, 1800.0, 2000.0);

    private static final List<Double> hoods =
            List.of(.5, .55, .6, .65, .7, .55, .65);

//    public InterpLUT shooterILUT;
//    public InterpLUT hoodILUT;
    public static final Interpolation2D closeInterpolation = new BilinearInterpolation(xs, ys, closeVelocities);
    private double velocity;

    public Shooter(HardwareMap hardwareMap) {
        h = hardwareMap.get(Servo.class, "h");
        l = new CachedMotor(hardwareMap.get(DcMotorEx.class, "sl"));
        r = new CachedMotor(hardwareMap.get(DcMotorEx.class, "sr"));
        l.setDirection(DcMotorSimple.Direction.REVERSE);

//        shooterILUT = new InterpLUT(distances, velocities);
//        hoodILUT = new InterpLUT(distances, hoods);
    }

    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setPower(double p) {
        l.setPower(p);
        r.setPower(p);
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public void shooterToggle() {
        activated = !activated;
        if (!activated)
            setPower(0);
    }

    public void setTarget(double velocity) {
        t = velocity;
    }

    public void periodic() {
        if (activated) {
            velocity = -l.getVelocity();

            if (Math.abs(getTarget() - getVelocity()) > useRaw)
                setPower(Math.signum(getTarget() - getVelocity()));
            else
                setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);
        }
    }

    public boolean atTarget() {
        return Math.abs((getTarget() - getVelocity())) < 50;
    }

    public void forPose(Pose current, Pose target, boolean close) {
        double xDistance = Math.abs(target.getX() - current.getX());

        if (close)
            setTarget(closeInterpolation.interpolate(xDistance, current.getY()));
        else
            setTarget(2000); // make a far interpolation
    }

    public void forDistance(double distance, boolean close) {
        if (!close) {
            setTarget(2000);
            setHood(.6);
        } else {
//            if (distance < 120 && distance > 35) {
//                setHood(hoodILUT.get(distance));
//                setTarget(shooterILUT.get(distance));
//            } else {
            //setTarget(953.09995 * Math.pow(distance, 0.166662));
            //setTarget((0.00255451 * Math.pow(distance, 3)) - (0.557218 * Math.pow(distance, 2)) + (41.91515 * distance) + 805.00963);
//            double hood = (-0.000327755 * Math.pow(distance, 2)) + (0.0391562 * distance) - 0.482427;
//
//            if (hood >= .77)
//                hood = .77;
//
//            if (hood <= .22)
//                hood = .22;
//
//            setHood(hood);
//            }

            if (distance > 90) {
                setHood(.5);
                setTarget(1800);
            }

            setHood(.55);
            setTarget(1650);

        }
    }

    public double timeInAir() {
        return 1.0;
    }

    public void setHood(double p) {
        h.setPosition(p);
    }

    public void close() {
        setTarget(1650);
        setHood(0.55);
        on();
    }

    public static Pose getProjectedPoseWithConstantVelocity(
            Pose initialPose,
            double time,
            Pose velocity
    ) {
        double vx = velocity.getX();
        double vy = velocity.getY();
        double omega = velocity.getHeading();

        double theta0 = initialPose.getHeading();

        // Pure translation (limit ω → 0)
        if (Math.abs(omega) < 1e-9) {
            double dx = (vx * Math.cos(theta0) - vy * Math.sin(theta0)) * time;
            double dy = (vx * Math.sin(theta0) + vy * Math.cos(theta0)) * time;

            return new Pose(
                    initialPose.getX() + dx,
                    initialPose.getY() + dy,
                    theta0
            );
        }

        double theta = omega * time;
        double sinT = Math.sin(theta);
        double cosT = Math.cos(theta);

        // V(theta) * v   (body → local frame)
        double dxLocal =
                ( sinT * vx - (1 - cosT) * vy ) / omega;
        double dyLocal =
                ( (1 - cosT) * vx + sinT * vy ) / omega;

        // Rotate into world frame
        double cos0 = Math.cos(theta0);
        double sin0 = Math.sin(theta0);

        double dxWorld = cos0 * dxLocal - sin0 * dyLocal;
        double dyWorld = sin0 * dxLocal + cos0 * dyLocal;

        return new Pose(
                initialPose.getX() + dxWorld,
                initialPose.getY() + dyWorld,
                theta0 + theta
        );
    }

    private Pose iterateOffset(Pose targetPos, Pose currentPos, Pose fieldVelocity) {
        Matrix inverseRotation = rotMatrix(currentPos.getHeading()).transposed();
        Vector velVector = fieldVelocity.getAsVector();
        Vector robotLinearVel = velVector.transform(inverseRotation);
        Pose robotVelPose = new Pose(robotLinearVel.getXComponent(), robotLinearVel.getYComponent(), fieldVelocity.getHeading());

        Pose currentIteration = targetPos.minus(currentPos);
        for (int i = 0; i < 10; i++) {
//            Pose disp = currentIteration.minus(currentPos);
            double shotTime = 1; //airTime.interpolate(currentIteration.getX(), currentIteration.getY());
            currentIteration = targetPos.minus(getProjectedPoseWithConstantVelocity(
                    currentPos,
                    shotTime,
                    robotVelPose
            ));
        }

        return currentIteration;
    }

    public static Matrix rotMatrix(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double[][] vals = new double[][] {
                {cos, -sin},
                {-sin, cos}
        };
        return new Matrix(vals);
    }

}

