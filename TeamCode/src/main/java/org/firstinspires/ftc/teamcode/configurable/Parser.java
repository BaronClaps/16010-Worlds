package org.firstinspires.ftc.teamcode.configurable;

import android.content.Context;
import android.content.res.AssetManager;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.pedropathing.ivy.Command;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import dev.frozenmilk.sinister.Scanner;
import dev.frozenmilk.sinister.sdk.apphooks.OnCreateEventLoop;
import dev.frozenmilk.sinister.sdk.opmodes.SinisterRegisteredOpModes;
import dev.frozenmilk.sinister.targeting.EmptySearch;
import dev.frozenmilk.sinister.targeting.SearchTarget;
import dev.frozenmilk.util.graph.Graph;
import dev.frozenmilk.util.graph.rule.AdjacencyRule;
import dev.frozenmilk.util.graph.rule.AdjacencyRules;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class Parser implements OnCreateEventLoop {
    public static OpModeManagerImpl opModeManager;
    public static Context context;
    //public static final Map<Class<?>, TemplateData> TEMPLATE_DATA = new LinkedHashMap<>();
    public static TemplateData templateData;

    private Parser() {
    }

    public static void registerAllOpModes() throws IOException {
        //get the yaml in the folder of /autos
        //match each yaml auto's template name to a class discovered by Sinister

        ObjectMapper mapper = new ObjectMapper(new YAMLFactory());
        AssetManager assets = context.getAssets();

        String[] assetList = assets.list("autos");
        if (assetList == null) return;

        for (String asset : assetList) {
            if (!asset.endsWith(".yml") && !asset.endsWith(".yaml")) continue;

            String assetPath = "autos/" + asset;

            try (InputStream is = assets.open(assetPath)) {
                ParsingConfig config = mapper.readValue(is, ParsingConfig.class);

                if (templateData == null) {
                    continue;
                }

                for (String command : config.getCommands()) {
                    Method method = templateData.steps.get(command);
                    if (method != null && method.getName().equalsIgnoreCase(command)) {
                        try {
                            Command cmd = (Command) method.invoke(null);
                            config.addCommand(cmd);
                        } catch (IllegalAccessException | java.lang.reflect.InvocationTargetException e) {
                            throw new RuntimeException(e);
                        }
                    }
                }

                for (Method m: templateData.variants) {
                    String variantLabel = m.getName();
                    Auto.Variant ann = m.getAnnotation(Auto.Variant.class);
                    if (ann != null) {
                        try {
                            Method annValue = ann.annotationType().getMethod("value");
                            Object val = annValue.invoke(ann);
                            variantLabel = String.valueOf(val);
                        } catch (Exception ignored) {}
                    }

                    OpModeMeta meta = new OpModeMeta.Builder()
                            .setName(variantLabel + " " + config.getName())
                            .setGroup(String.valueOf(config.getPriority()))
                            .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                            .setSource(OpModeMeta.Source.EXTERNAL_LIBRARY)
                            .build();

                    SinisterRegisteredOpModes.INSTANCE.register(meta, (OpMode) m.invoke(null));
                }
            } catch (JsonProcessingException ignored) {} catch (InvocationTargetException | IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static class TemplateData {
        public final Map<String, Method> steps = new LinkedHashMap<>();
        public final List<Method> variants = new ArrayList<>();
    }

    private static final class TemplateScanner implements Scanner {
        public static final TemplateScanner INSTANCE = new TemplateScanner();
        private static final SearchTarget FILTER_SEARCH_TARGET = new EmptySearch().include("org.firstinspires.ftc.teamcode.configurable.templates");

        private TemplateScanner() {}

        @NotNull
        @Override
        public SearchTarget getTargets() {
            return FILTER_SEARCH_TARGET;
        }

        @Override
        public void scan(@NotNull ClassLoader loader, @NotNull Class<?> cls) {
            if (!cls.isAnnotationPresent(Auto.Template.class)) return;

            TemplateData info = new TemplateData();

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

    @Override
    public void onCreateEventLoop(@NotNull Context context, @NotNull FtcEventLoop ftcEventLoop) {
        opModeManager = ftcEventLoop.getOpModeManager();
        Parser.context = context;
        try {
            registerAllOpModes();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
