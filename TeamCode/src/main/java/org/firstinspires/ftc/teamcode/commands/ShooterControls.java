package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ShooterControls {

    private final Shoot shoot;

    private double targetRPM = 250;
    private double targetVelocity = 1725;

    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    public ShooterControls(Shoot shoot) {
        this.shoot = shoot;
    }

    public void update(Gamepad gamepad, double distance) {

        // Shooter and intake
        if (gamepad.right_bumper) {

            double velocity = Calculator.getTargetVelocity(distance);

            if (gamepad.right_trigger > 0) {
                shoot.shootWithIntake(velocity);
            } else {
                shoot.shoot(velocity);
            }

        } else if (gamepad.left_trigger > 0) {

            shoot.reverseShooter();

        } else if (gamepad.left_bumper) {

            shoot.reverseIntake();

        } else {

            shoot.stop();
        }

        // RPM adjustment
        if (gamepad.dpad_left && !dpadLeftPressed) {
            targetRPM += 250;
        } else if (gamepad.dpad_right && !dpadRightPressed) {
            targetRPM -= 250;
        }

        dpadLeftPressed = gamepad.dpad_left;
        dpadRightPressed = gamepad.dpad_right;
    }

    public void stop() {
        shoot.stop();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }
}