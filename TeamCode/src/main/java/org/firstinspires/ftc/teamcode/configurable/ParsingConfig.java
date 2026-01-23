package org.firstinspires.ftc.teamcode.configurable;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.groups.Groups;

import java.util.List;

import static com.pedropathing.ivy.groups.Groups.sequential;

public class ParsingConfig {
    private String name;
    private int priority = 0;
    private List<String> commands;
    private List<Command> cmds;

    public Command get() {
        // this would just be turned the actual scanned mapped commands!
        return sequential(cmds.toArray(new Command[0]));
    }

    public List<String> getCommands() {
        return commands;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getPriority() {
        return priority;
    }

    public void setPriority(int priority) {
        this.priority = priority;
    }

    public void addCommand(Command command) {
        this.cmds.add(command);
    }
}
