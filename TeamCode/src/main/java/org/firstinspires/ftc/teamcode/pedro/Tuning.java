package org.firstinspires.ftc.teamcode.pedro;
import static org.firstinspires.ftc.teamcode.pedro.Constants.drivetrainConfig;
import static org.firstinspires.ftc.teamcode.pedro.Constants.localizerConfig;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

public class Tuning {
    // Tuners go here
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, drivetrainConfig), null, null);
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    // I finished pinpointTuner, we need to configure foresight later
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, drivetrainConfig));
    }
    // Also do follower
    /*
    public static Follower create(HardwareMap h) {
    return new Follower(
        new PinpointLocalizer(h, localizerConfig),
        new Mecanum(h, drivetrainConfig),
        new Foresight(foresightConfig)
    );
    }
     */
}
