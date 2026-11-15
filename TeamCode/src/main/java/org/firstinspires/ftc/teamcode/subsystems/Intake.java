package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intake;

    public Intake(HardwareMap hardwareMap) {

        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotor.Direction.FORWARD);
        stop();
    }

    public void setPower(double power) {
        intake.setPower(power);
    }

    public void intake() {
        setPower(0.35);
    }

    public void reverse() {
        setPower(-0.1);
    }

    public void stop() {
        setPower(0);
    }
}
