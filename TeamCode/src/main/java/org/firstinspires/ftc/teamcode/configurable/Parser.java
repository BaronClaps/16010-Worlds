package org.firstinspires.ftc.teamcode.configurable;

import android.content.Context;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import dev.frozenmilk.sinister.sdk.apphooks.OnCreateEventLoop;
import dev.frozenmilk.sinister.sdk.opmodes.SinisterRegisteredOpModes;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.jetbrains.annotations.NotNull;

import java.util.Objects;

public class Parser implements OnCreateEventLoop {
    public static final Parser INSTANCE = new Parser();
    public static OpModeManagerImpl opModeManager;

    private Parser() {
    }

    public static void registerAllOpModes() {
        //get the yaml in the folder of /autos
        //match each yaml auto's template name to an opmode class with @AutoTemplate(templateName)
        // grab each scanned opmode's variants from the method
        // register them all

//        for (Class<? extends OpMode> opmode : ) {
//        }
//            OpModeMeta meta = new OpModeMeta.Builder()
//                    .setName("Test OpMode")
//                    .setFlavor(OpModeMeta.Flavor.TELEOP)
//                    .setSource(OpModeMeta.Source.EXTERNAL_LIBRARY)
//                    .build();
//            SinisterRegisteredOpModes.INSTANCE.register(meta, opmode.class );
//            opModeManager.initOpMode("Test OpMode");
//            opModeManager.startActiveOpMode();
    }

    @Override
    public void onCreateEventLoop(@NotNull Context context, @NotNull FtcEventLoop ftcEventLoop) {
        opModeManager = ftcEventLoop.getOpModeManager();
        registerAllOpModes();
    }
}