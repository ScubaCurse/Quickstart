package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class Shoot {

    private Flywheel flywheel;
    private Intake intake;

    public Shoot(Flywheel flywheel, Intake intake) {
        this.flywheel = flywheel;
        this.intake = intake;
    }

    public void shoot(double velocity) {
        flywheel.setVelocity(velocity);
        intake.stop();
    }

    public void shootWithIntake(double velocity) {
        flywheel.setVelocity(velocity);
        intake.intake();
    }

    public void reverseShooter() {
        flywheel.reverse();
        intake.setPower(1);
    }

    public void reverseIntake() {
        flywheel.stop();
        intake.reverse();
    }

    public void stop() {
        flywheel.stop();
        intake.stop();
    }
    public void shootForTime(double targetVelocity, double seconds,
                             LinearOpMode opMode) {

        flywheel.setVelocity(targetVelocity);

        opMode.sleep(1000);
            // Feed POLLEN
            intake.setPower(0.35);
            opMode.sleep((long)(seconds * 1000));
            stop();
    }
}
