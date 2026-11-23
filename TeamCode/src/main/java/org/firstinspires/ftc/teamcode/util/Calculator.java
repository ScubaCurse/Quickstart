package org.firstinspires.ftc.teamcode.util;

public class Calculator {

    private static final double[] distances = {
            // in inches, 30.48 - 243.84 cm
            24, 34, 48, 60, 72, 84
    };

    private static final double[] velocities = {
            // in encoder ticks per second
            1650, 1750, 1875, 1925, 2175, 2300
    };

    public static double getTargetVelocity(double distance) {

        if (distance <= distances[0]) {
            return velocities[0];
        }

        if (distance >= distances[distances.length - 1]) {
            return velocities[velocities.length - 1];
        }

        for (int i = 0; i < distances.length - 1; i++) {

            if (distance <= distances[i + 1]) {

                double ratio =
                        (distance - distances[i]) /
                                (distances[i + 1] - distances[i]);

                return velocities[i] +
                        ratio * (velocities[i + 1] - velocities[i]);
            }
        }

        return velocities[velocities.length - 1];
    }

    public static double tpsToRpm(double tps, double cer) {
        // return RPM
        return (tps * 60) / cer;
    }

    public static double rpmToTps(double rpm, double cer) {
        // return TPS
        return (rpm * cer) / 60;
    }
}
