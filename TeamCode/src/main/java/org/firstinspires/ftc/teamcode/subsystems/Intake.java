package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intake;

    public Intake(HardwareMap hardwareMap) {

        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotor.Direction.FORWARD);
        stop();
    }

    public void setPower(double power) {
        intake.setPower(power);
    }

    public void stop() {
        setPower(0);
    }
}
