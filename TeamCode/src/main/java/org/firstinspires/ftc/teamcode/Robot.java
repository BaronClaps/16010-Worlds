package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class Robot {
    private HardwareMap hardwareMap;
    private Alliance alliance;
    public Robot(HardwareMap h, Alliance a) {
        this.hardwareMap = h;
        this.alliance = a;
    }
}
