package org.firstinspires.ftc.teamcode.configurable;

import com.pedropathing.ivy.Command;

import java.util.List;

import static com.pedropathing.ivy.groups.Groups.sequential;

public class ParsingConfig {
    private List<Command> commands;

    public Command get() {
        return sequential(commands.toArray(new Command[0]));
    }
    public List<Command> getCommands() {
        return commands;
    }
}
