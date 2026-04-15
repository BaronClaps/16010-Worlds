package org.firstinspires.ftc.teamcode.variant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Close;
import org.firstinspires.ftc.teamcode.Far;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous
public class RedFar extends Far {
    public RedFar() {
        super(Alliance.RED);
    }
}
