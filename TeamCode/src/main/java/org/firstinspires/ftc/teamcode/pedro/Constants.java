package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    // Please update MecanumConfig and Pinpoint
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("RL");
        c.backRightName.set("RR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.8008935883289248);
        c.yPodOffset.set(-6.000945324034203);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2417743638010892);
                Controller secondaryTranslationalForward = Controller.proportional(0.08932916709452009);
                Controller primaryTranslationalLateral = Controller.proportional(0.40628176325370013);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15011025547358936);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012571695704620706));
                c.brake.set(Controller.proportionalFeedforward(0.0106859413489276));

                c.headingFeedback.set(Controller.proportional(2.785099268521964));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06835278610831942, 0.0025722891914098087));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09255543916854997, 0.07524283631206788));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011715688826578123, 0.0015004605486256602));

                c.maxAchievableForwardVelocity.set(75.59565634509674);
                c.maxAchievableStrafeVelocity.set(59.60429067284245);
                c.naturalForwardDeceleration.set(29.775504477456987);
                c.naturalStrafeDeceleration.set(59.172298212090794);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}