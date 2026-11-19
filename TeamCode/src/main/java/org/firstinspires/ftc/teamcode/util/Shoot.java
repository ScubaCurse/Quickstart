
package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class Shoot {

    private final Flywheel flywheel;
    private final Intake intake;

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

    public Command shootForTime(double target, double seconds) {
        ElapsedTime timer = new ElapsedTime();

        return Command.build()
                .setStart(timer::reset)
                .setExecute(() -> {
                    flywheel.setVelocity(target);
                    if (timer.seconds() >= seconds) {
                        intake.intake();
                    }
                })
                .setDone(() -> timer.seconds() >= seconds + 1)
                .setEnd(endCondition -> {
                    flywheel.stop();
                    intake.stop();
                })
                .requiring(flywheel, intake);
    }

    public Command timedIntake(double power, double seconds) {
        ElapsedTime timer = new ElapsedTime();

        return Command.build()
                .setStart(timer::reset)
                .setExecute(() -> intake.setPower(power))
                .setDone(() -> timer.seconds() >= seconds)
                .setEnd(endCondition -> intake.stop())
                .requiring(intake);
    }

    public Command timedReverseIntake(double power, double seconds) {
        ElapsedTime timer = new ElapsedTime();

        return Command.build()
                .setStart(timer::reset)
                .setExecute(() -> intake.setPower(-power))
                .setDone(() -> timer.seconds() >= seconds)
                .setEnd(endCondition -> intake.stop())
                .requiring(intake);
    }
}
