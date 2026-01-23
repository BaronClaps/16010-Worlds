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
import dev.frozenmilk.sinister.sdk.apphooks.OnCreateEventLoop;
import dev.frozenmilk.sinister.sdk.opmodes.SinisterRegisteredOpModes;
import dev.frozenmilk.sinister.targeting.EmptySearch;
import dev.frozenmilk.sinister.targeting.SearchTarget;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import dev.frozenmilk.sinister.filtering.SinisterFilter;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
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
                        } catch (Exception ignored) {
                        }
                    }

                    OpModeMeta meta = new OpModeMeta.Builder()
                            .setName(variantLabel + " " + config.getName())
                            .setGroup(String.valueOf(config.getPriority()))
                            .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                            .setSource(OpModeMeta.Source.EXTERNAL_LIBRARY)
                            .build();

                    SinisterRegisteredOpModes.INSTANCE.register(meta, (OpMode) m.invoke(config));
                }

//                for (Class<?> template : TEMPLATE_DATA.keySet()) {
//                    if (template.getClass().equals(Template.class)) {

            } catch (JsonProcessingException ignored) {} catch (InvocationTargetException | IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static class TemplateData {
        public final Map<String, Method> steps = new LinkedHashMap<>();
        public final List<Method> variants = new ArrayList<>();
    }

    private static final class TemplateFilter implements SinisterFilter {
        public static final TemplateFilter INSTANCE = new TemplateFilter();
        private static final SearchTarget FILTER_SEARCH_TARGET = new EmptySearch().include("org.firstinspires.ftc.teamcode.configurable.templates");

        private TemplateFilter() {}

        @Override
        public SearchTarget getTargets() {
            return FILTER_SEARCH_TARGET;
        }

        @Override
        public void filter(@NotNull Class<?> clazz) {
            if (clazz.isAnnotationPresent(Auto.Template.class)) {
                TemplateData info = new TemplateData();
                for (Method m : clazz.getDeclaredMethods()) {
                    if (m.isAnnotationPresent(Auto.Step.class)) {
                        info.steps.put(m.getName(), m);
                    }
                    if (m.isAnnotationPresent(Auto.Variant.class)) {
                        info.variants.add(m);
                    }
                }
            }
        }
    }

    private static String signatureOf(Method m) {
        StringBuilder sb = new StringBuilder();
        sb.append(m.getReturnType().getTypeName()).append(' ').append(m.getName()).append('(');
        Class<?>[] params = m.getParameterTypes();
        for (int i = 0; i < params.length; i++) {
            if (i > 0) sb.append(',');
            sb.append(params[i].getTypeName());
        }
        sb.append(')');
        return sb.toString();
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
