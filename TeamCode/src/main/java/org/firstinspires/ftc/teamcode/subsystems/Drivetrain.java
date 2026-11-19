package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    private final DcMotor FL;
    private final DcMotor RL;
    private final DcMotor FR;
    private final DcMotor RR;

    public Drivetrain(HardwareMap hardwareMap) {

        FL = hardwareMap.get(DcMotor.class, "FL");
        RL = hardwareMap.get(DcMotor.class, "RL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        RR = hardwareMap.get(DcMotor.class, "RR");

        FL.setDirection(DcMotor.Direction.FORWARD);
        RL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.REVERSE);
    }

    public void mecanum(double axial, double lateral, double yaw) {

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        frontLeftPower = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;
        backLeftPower = axial + lateral - yaw;
        backRightPower = axial - lateral + yaw;

        // Normalize motor powers
        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (max > 1) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Set motor powers
        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        RL.setPower(backLeftPower);
        RR.setPower(backRightPower);
    }

    public void stop() {
        mecanum(0, 0, 0);
    }
}
