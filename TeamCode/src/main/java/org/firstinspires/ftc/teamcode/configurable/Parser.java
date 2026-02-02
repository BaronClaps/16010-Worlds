//package org.firstinspires.ftc.teamcode.configurable;
//
//import android.content.Context;
//import android.content.res.AssetManager;
//import android.util.Log;
//import com.fasterxml.jackson.core.JsonProcessingException;
//import com.fasterxml.jackson.databind.ObjectMapper;
//import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
//import com.pedropathing.ivy.Command;
//import com.qualcomm.ftccommon.FtcEventLoop;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
//import dev.frozenmilk.sinister.Scanner;
//import dev.frozenmilk.sinister.loading.Preload;
//import dev.frozenmilk.sinister.sdk.apphooks.OnCreateEventLoop;
//import dev.frozenmilk.sinister.sdk.opmodes.SinisterRegisteredOpModes;
//import dev.frozenmilk.sinister.targeting.EmptySearch;
//import dev.frozenmilk.sinister.targeting.SearchTarget;
//import dev.frozenmilk.sinister.targeting.TeamCodeSearch;
//import dev.frozenmilk.util.graph.Graph;
//import dev.frozenmilk.util.graph.rule.AdjacencyRule;
//import dev.frozenmilk.util.graph.rule.AdjacencyRules;
//import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
//import org.jetbrains.annotations.NotNull;
//
//import java.io.IOException;
//import java.io.InputStream;
//import java.lang.reflect.InvocationTargetException;
//import java.lang.reflect.Method;
//
//public class Parser implements OnCreateEventLoop {
//    private final static Parser INSTANCE = new Parser();
//    public static OpModeManagerImpl opModeManager;
//    public static Context context;
//    public static TemplateData templateData;
//
//    private Parser() {
//    }
//
//    public static void registerAllOpModes() {
//        //get the yaml in the folder of /autos
//        //match each yaml auto's template name to a class discovered by Sinister
//
//        OpModeMeta meta12 = new OpModeMeta.Builder()
//                .setName("e")
//                .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
//                .setSource(OpModeMeta.Source.EXTERNAL_LIBRARY)
//                .build();
//
//        SinisterRegisteredOpModes.INSTANCE.register(meta12, Template.getBlue(new ParsingConfig()));
//
//        try {
//            ObjectMapper mapper = new ObjectMapper(new YAMLFactory());
//            AssetManager assets = context.getAssets();
//
//            String[] assetList = assets.list("autos");
//            if (assetList == null) return;
//
//            Log.d("Parser", "Assets Found: " + String.join(", ", assetList));
//
//            for (String asset : assetList) {
//                if (!asset.endsWith(".yaml")) continue;
//
//                Log.d("Parser", "Found YAML: " + asset);
//
//                String assetPath = "autos/" + asset;
//
//                try (InputStream is = assets.open(assetPath)) {
//                    ParsingConfig config = mapper.readValue(is, ParsingConfig.class);
//
//                    if (templateData == null) {
//                        Log.e("Parser", "No template data found, skipping...");
//                        continue;
//                    }
//
//                    Log.d("Parser", "Processing Config: " + config.getName());
//
//                    for (String command : config.getCommands()) {
//                        Method method = templateData.steps.get(command);
//                        if (method != null && method.getName().equalsIgnoreCase(command)) {
//                            try {
//                                Command cmd = (Command) method.invoke(null);
//                                config.addCommand(cmd);
//                            } catch (IllegalAccessException | java.lang.reflect.InvocationTargetException e) {
//                                throw new RuntimeException(e);
//                            }
//                        }
//                    }
//
//                    for (Method m : templateData.variants) {
//                        String variantLabel = m.getName();
//                        Auto.Variant ann = m.getAnnotation(Auto.Variant.class);
//                        if (ann != null) {
//                            try {
//                                Method annValue = ann.annotationType().getMethod("value");
//                                Object val = annValue.invoke(ann);
//                                variantLabel = String.valueOf(val);
//                            } catch (Exception ignored) {
//                            }
//                        }
//
//                        Log.d("Parser", "Registering Variant: " + variantLabel);
//
//                        OpModeMeta meta = new OpModeMeta.Builder()
//                                .setName(variantLabel + " " + config.getName())
//                                .setGroup(String.valueOf(config.getPriority()))
//                                .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
//                                .setSource(OpModeMeta.Source.EXTERNAL_LIBRARY)
//                                .build();
//
//                        SinisterRegisteredOpModes.INSTANCE.register(meta, (OpMode) m.invoke(null));
//                    }
//                } catch (JsonProcessingException ignored) {
//                } catch (InvocationTargetException | IllegalAccessException e) {
//                    Log.e("Parser", "Error processing config", e);
//                    throw new RuntimeException(e);
//                }
//            }
//        } catch (RuntimeException | IOException e) {
//            throw new RuntimeException(e);
//        }
//
//        OpModeMeta meta1 = new OpModeMeta.Builder()
//                .setName("ee")
//                .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
//                .setSource(OpModeMeta.Source.EXTERNAL_LIBRARY)
//                .build();
//
//        SinisterRegisteredOpModes.INSTANCE.register(meta1, Template.getBlue(new ParsingConfig()));
//    }
//
//    @Override
//    public void onCreateEventLoop(@NotNull Context context, @NotNull FtcEventLoop ftcEventLoop) {
//        opModeManager = ftcEventLoop.getOpModeManager();
//        Parser.context = context;
//        Log.println(Log.ASSERT, "onCreateEventLoop", "Created Parser");
//        registerAllOpModes();
//    }
//}
//
