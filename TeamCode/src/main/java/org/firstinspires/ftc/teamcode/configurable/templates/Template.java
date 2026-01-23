package org.firstinspires.ftc.teamcode.configurable.templates;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.configurable.Auto;
import org.firstinspires.ftc.teamcode.configurable.ParsingConfig;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

import java.util.ArrayList;
import java.util.List;

@Auto.Template
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

    @Auto.Step
    public Command intake1Close() {
        return Command.NOOP;
    }

    @Auto.Variant("Red")
    public OpMode getRed(ParsingConfig parsingConfig) {
        return new Template(Alliance.RED, parsingConfig);
    }

    @Auto.Variant("Blue")
    public OpMode getBlue(ParsingConfig parsingConfig) {
        return new Template(Alliance.BLUE, parsingConfig);
    }
}
