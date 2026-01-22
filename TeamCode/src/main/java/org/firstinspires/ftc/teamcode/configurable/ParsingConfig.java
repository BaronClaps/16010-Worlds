package org.firstinspires.ftc.teamcode.configurable;

import com.pedropathing.ivy.Command;

import java.util.List;

import static com.pedropathing.ivy.groups.Groups.sequential;

public class ParsingConfig {
    private String name;
    private int priority = 0;
    private List<String> commands;

    public Command get() {
        // this would just be turned the actual scanned mapped commands!
        return Command.NOOP; //sequential(commands.toArray(new Command[0]));
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
}
