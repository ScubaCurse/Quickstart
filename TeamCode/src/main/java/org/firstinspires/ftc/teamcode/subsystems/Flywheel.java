package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {

    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;

    public Flywheel(HardwareMap hardwareMap) {

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        stop();
    }

    public void setVelocity(double velocity) {
        shooterLeft.setVelocity(velocity);
        shooterRight.setVelocity(velocity);
    }

    public void stop() {
        setVelocity(0);
    }

    public double getLeftVelocity() {
        return shooterLeft.getVelocity();
    }

    public double getRightVelocity() {
        return shooterRight.getVelocity();
    }

    public double getAverageVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }
}
