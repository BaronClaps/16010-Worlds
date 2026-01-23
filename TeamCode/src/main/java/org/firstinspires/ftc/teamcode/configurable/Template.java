package org.firstinspires.ftc.teamcode.configurable;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

import java.util.ArrayList;
import java.util.List;

public class Template extends CommandOpMode {
    Alliance a;
    ParsingConfig parsingConfig;
    Robot r;

    public Template(Alliance a, ParsingConfig parsingConfig) {
        this.a = a;
        this.parsingConfig = parsingConfig;
    }

    public void init() {
        r = new Robot(hardwareMap, a);
        schedule(parsingConfig.get());
    }

    public Command intake1Close() {
        return Command.NOOP;
    }

    public List<OpMode> getVariants() {
        return new ArrayList<OpMode>();
    }

    @Auto.Variant
    public OpMode getRed(ParsingConfig parsingConfig) {
        return new Template(Alliance.RED, parsingConfig);
    }

    @Auto.Variant
    public OpMode getBlue(ParsingConfig parsingConfig) {
        return new Template(Alliance.BLUE, parsingConfig);
    }
}
