package org.firstinspires.ftc.teamcode.configurable;

import android.content.Context;
import android.content.res.AssetManager;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import dev.frozenmilk.sinister.sdk.apphooks.OnCreateEventLoop;
import dev.frozenmilk.sinister.sdk.opmodes.SinisterRegisteredOpModes;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.io.InputStream;
import java.util.LinkedHashMap;
import java.util.Map;

public class Parser implements OnCreateEventLoop {
    public static final Parser INSTANCE = new Parser();
    public static OpModeManagerImpl opModeManager;
    public static Context context;

    private Parser() {
    }

    public static void registerAllOpModes() throws IOException {
        //get the yaml in the folder of /autos
        //match each yaml auto's template name to an opmode class with @AutoTemplate(templateName)
        // grab each scanned opmode's variants from the method
        // register them all


        // get the yaml files in the assets/autos folder, parse them into ParsingConfig
        // and register a simple OpModeMeta for each parsed file (using filename as the op mode name).
        ObjectMapper mapper = new ObjectMapper(new YAMLFactory());
        AssetManager assets = context.getAssets();

        String[] assetList = assets.list("autos");
        if (assetList == null) return;

        for (String asset : assetList) {
            if (!asset.endsWith(".yml") && !asset.endsWith(".yaml")) continue;

            String assetPath = "autos/" + asset;
            try (InputStream is = assets.open(assetPath)) {
                ParsingConfig config = mapper.readValue(is, ParsingConfig.class);

//                OpModeMeta meta = new OpModeMeta.Builder()
//                        .setName("name")
//                        .setGroup("Configurable Autos")
//                        .setSource(OpModeMeta.Source.EXTERNAL_LIBRARY)
//                        .build();

            } catch (JsonProcessingException e) {
                System.out.println("Failed to parse: " + asset + " -> " + e.getMessage());
            }
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