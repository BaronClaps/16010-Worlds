package org.firstinspires.ftc.teamcode.configurable;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;

import java.util.Arrays;
import java.util.List;

public class Template extends CommandOpMode {
    List<Command> commands;
    Alliance a;
    Robot r;

    public Template(Alliance a, String parseable) {
        this.a = a;
        ObjectMapper mapper = new ObjectMapper(new YAMLFactory());

        try {
            commands = mapper.readValue(parseable, ParsingConfig.class).getCommands();
        } catch (JsonProcessingException e) {}
    }

    public void init() {
        r = new Robot(hardwareMap, a);
    }

    @AutoStep
    public Command intake1Close() {
        return Command.NOOP;
    }
}
