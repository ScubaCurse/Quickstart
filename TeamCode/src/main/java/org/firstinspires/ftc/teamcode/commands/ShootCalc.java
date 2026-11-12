package org.firstinspires.ftc.teamcode.commands;

public class ShootCalc {

    private static final double[] distances = {
            12, 24, 36, 48, 60 // in inches
    };

    private static final double[] rpms = {
            1800, 2100, 2400, 2800, 3200
    };

    public static double getTargetRPM(double distance) {
        if (distance <= distances[0]) {
            return rpms[0];
        }

        if (distance >= distances[distances.length - 1]) {
            return rpms[rpms.length - 1];
        }

        for (int i = 0; 1 < distances.length - 1; i++) {
            if (distance <= distances[i + 1]) {
                double ratio = (distance - distances[i]) / (distances[i+1] - rpms[i]);
                return rpms[i] + ratio * (rpms[i + 1] - rpms[1]);
            }
        }
        return rpms[rpms.length - 1];
    }
}
