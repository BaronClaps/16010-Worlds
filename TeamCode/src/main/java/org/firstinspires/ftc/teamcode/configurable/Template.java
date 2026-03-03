package org.firstinspires.ftc.teamcode.configurable;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

import static com.pedropathing.ivy.groups.Groups.sequential;

@Auto.Template
public class Template extends CommandOpMode {
    Alliance a;
    static Telemetry t;
    ParsingConfig parsingConfig;
    
    Robot r;

    public Template(Alliance a, ParsingConfig parsingConfig) {
        this.a = a;
        this.parsingConfig = parsingConfig;
        Template.t = telemetry;
    }

    public void init() {
        schedule(parsingConfig.get());
    }

    @Auto.Step
    public static Command intake1Close() {
        return sequential(
                Commands.instant(() -> {
                    t.addLine("Intake 1 Close!");
                    t.update();
                }),
                Commands.waitMs(2000.0)
        );
    }

    @Auto.Step
    public static Command intake2Close() {
        return sequential(
                Commands.instant(() -> {
                    t.addLine("Intake 2 Close!");
                    t.update();
                }),
                Commands.waitMs(2000.0)
        );
    }

    @Auto.Step
    public static Command shootClose() {
        return sequential(
                Commands.instant(() -> {
                    t.addLine("Shooting Close!");
                    t.update();
                }),
                Commands.waitMs(2000.0)
        );
    }

    @Auto.Variant("Red")
    public static OpMode getRed(ParsingConfig parsingConfig) {
        return new Template(Alliance.RED, parsingConfig);
    }

    @Auto.Variant("Blue")
    public static OpMode getBlue(ParsingConfig parsingConfig) {
        return new Template(Alliance.BLUE, parsingConfig);
    }
}
