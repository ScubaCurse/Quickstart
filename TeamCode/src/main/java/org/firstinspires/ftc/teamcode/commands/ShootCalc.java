package org.firstinspires.ftc.teamcode.commands;

public class ShootCalc {

    private static final double[] distances = {
            // in inches, 30.48 - 243.84 cm
            12, 24, 36, 48, 60, 72, 84, 96
    };

    private static final double[] velocities = {
            // in encoder ticks per second
            0, 0, 1725, 0, 0, 0, 0, 0
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
}
