package org.firstinspires.ftc.teamcode.configurable;

import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

import java.util.Arrays;
import java.util.List;

public class Template extends CommandOpMode {
    List<Command> commands;
    Alliance a;

    public Template(Alliance a, Command... commands) {
        this.a = a;
        this.commands = Arrays.asList(commands);
    }

    public void init() {
    }
}
