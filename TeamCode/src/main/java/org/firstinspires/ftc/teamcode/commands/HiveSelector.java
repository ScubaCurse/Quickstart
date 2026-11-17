package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

public class HiveSelector {

    private double hiveX = 57.5;
    private double hiveY = 96.9;
    private int hiveSelected = 2;

    public void update(Gamepad gamepad) {

        if (gamepad.xWasPressed()) {
            hiveSelected = 1;
            hiveX = 57.5;
            hiveY = 43.0;

        } else if (gamepad.yWasPressed()) {
            hiveSelected = 2;
            hiveX = 57.5;
            hiveY = 96.9;
        }
    }

    public double getDistance(double robotX, double robotY) {
        return Math.hypot(hiveX - robotX, hiveY - robotY);
    }

    public int getSelectedHive() {
        return hiveSelected;
    }
}