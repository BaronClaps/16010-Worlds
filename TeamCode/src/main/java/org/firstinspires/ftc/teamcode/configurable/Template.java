package org.firstinspires.ftc.teamcode.configurable;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;

import java.util.ArrayList;
import java.util.Arrays;
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

    @AutoStep
    public Command intake1Close() {
        return Command.NOOP;
    }

    public List<OpMode> getVariants() {
        return new ArrayList<OpMode>();
    }
}
