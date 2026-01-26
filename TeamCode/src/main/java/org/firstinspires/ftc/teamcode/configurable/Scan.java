package org.firstinspires.ftc.teamcode.configurable;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import dev.frozenmilk.sinister.Scanner;
import dev.frozenmilk.sinister.loading.Preload;
import dev.frozenmilk.sinister.targeting.SearchTarget;
import dev.frozenmilk.sinister.targeting.TeamCodeSearch;
import dev.frozenmilk.util.graph.Graph;
import dev.frozenmilk.util.graph.rule.AdjacencyRule;
import dev.frozenmilk.util.graph.rule.AdjacencyRules;
import org.jetbrains.annotations.NotNull;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
@Preload
 public class Scan implements Scanner {
    public static final Scan INSTANCE = new Scan();
    private static final SearchTarget FILTER_SEARCH_TARGET = new TeamCodeSearch();

    private Scan() {}

    @NotNull
    @Override
    public SearchTarget getTargets() {
        return FILTER_SEARCH_TARGET;
    }

    @Override
    public void scan(@NotNull ClassLoader loader, @NotNull Class<?> cls) {
        if (!cls.isAnnotationPresent(Auto.Template.class)) return;

        Parser.TemplateData info = new Parser.TemplateData();

        for (Method m : cls.getDeclaredMethods()) {
            if (m.isAnnotationPresent(Auto.Step.class)) {
                boolean isStatic = Modifier.isStatic(m.getModifiers());

                if (!isStatic) {
                    throw new IllegalArgumentException("Method " + cls.getCanonicalName() + "." + m.getName() +
                            " annotated with @Auto.Step must be static");
                }

                if (m.getParameterCount() != 0) {
                    throw new IllegalArgumentException("Method " + cls.getCanonicalName() + "." + m.getName() +
                            " annotated with @Auto.Step must have 0 parameters");
                }

                if (!Command.class.isAssignableFrom(m.getReturnType())) {
                    throw new IllegalArgumentException("Method " + cls.getCanonicalName() + "." + m.getName() +
                            " annotated with @Auto.Step must return com.pedropathing.ivy.Command");
                }

                m.setAccessible(true);
                info.steps.put(m.getName(), m);
            }

            if (m.isAnnotationPresent(Auto.Variant.class)) {
                boolean isStatic = Modifier.isStatic(m.getModifiers());

                if (!isStatic) {
                    throw new IllegalArgumentException("Method " + cls.getCanonicalName() + "." + m.getName() +
                            " annotated with @Auto.Variant must be static");
                }

                if (m.getParameterCount() != 0) {
                    throw new IllegalArgumentException("Method " + cls.getCanonicalName() + "." + m.getName() +
                            " annotated with @Auto.Variant must have 0 parameters");
                }

                if (!OpMode.class.isAssignableFrom(m.getReturnType())) {
                    throw new IllegalArgumentException("Method " + cls.getCanonicalName() + "." + m.getName() +
                            " annotated with @Auto.Variant must return an OpMode instance");
                }

                m.setAccessible(true);
                info.variants.add(m);
            }
        }

        if (Parser.templateData != null) {
            System.out.println("Multiple templates found; keeping first discovered, ignoring: " + cls.getName());
        } else {
            Parser.templateData = info;
            System.out.println("TemplateScanner discovered template: " + cls.getName() + " -> steps=" + info.steps.size() + " variants=" + info.variants.size());
        }
    }

    @Override
    public void unload(@NotNull ClassLoader loader, @NotNull Class<?> cls) {
        // no-op
    }

    @Override
    public @NotNull AdjacencyRule<Scanner, Graph<Scanner>> getLoadAdjacencyRule() {
        return AdjacencyRules.independent();
    }

    @Override
    public @NotNull AdjacencyRule<Scanner, Graph<Scanner>> getUnloadAdjacencyRule() {
        return AdjacencyRules.independent();
    }
}