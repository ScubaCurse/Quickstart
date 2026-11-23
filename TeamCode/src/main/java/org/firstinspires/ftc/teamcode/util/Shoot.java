package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class Shoot {
    private final Flywheel flywheel;
    private final Intake intake;

    public Shoot(HardwareMap hardwareMap) {
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public Command shootForTime(double target, double seconds) {
        if (seconds <= 0) {
            throw new IllegalArgumentException("seconds must be positive");
        }
        ElapsedTime timer = new ElapsedTime();

        return Command.build()
                .setStart(timer::reset)
                .setExecute(() -> {
                    flywheel.setVelocity(target);
                    if (timer.seconds() >= seconds - 1) {
                        intake.setPower(0.5);
                    }
                })
                .setDone(() -> timer.seconds() >= seconds)
                .setEnd(endCondition -> {
                    flywheel.stop();
                    intake.stop();
                })
                .requiring(flywheel, intake);
    }
}
