package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriverControls {

    private final Drivetrain drivetrain;
    private double speedMode = 0.01;

    public DriverControls(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void update(Gamepad gamepad) {

        double axial = -(gamepad.left_stick_y / speedMode);
        double lateral = gamepad.right_stick_x / 2;
        double yaw = gamepad.left_stick_x / speedMode;

        drivetrain.mecanum(axial, lateral, yaw);

        if (gamepad.dpad_up) {
            speedMode = 3;
        }

        if (gamepad.dpad_down) {
            speedMode = 0.01;
        }
    }

    public double getSpeedMode() {
        return speedMode;
    }
}