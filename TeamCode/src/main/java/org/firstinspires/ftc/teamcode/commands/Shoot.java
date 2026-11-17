package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class Shoot {

    private static Flywheel flywheel;
    private static Intake intake;

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
    public static Command shootForTime(double target, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(timer::reset)
                .setExecute(() -> {
                    if (timer.seconds() >= seconds) {
                        intake.setPower(0.35);
                    }
                })
                .setDone(() -> timer.seconds() >= seconds + 1)
                .setEnd(endCondition -> {
                    intake.setPower(0);
                    flywheel.setVelocity(0);
                })
                .requiring(flywheel, intake);
    }
    public static Command timedIntake(double power, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(timer::reset)
                .setExecute(() -> {
                    if (timer.seconds() >= seconds) {
                        intake.intake();
                    }
                })
                .setDone(() -> timer.seconds() >= seconds + 1)
                .setEnd(endCondition -> {
                    intake.setPower(0);
                })
                .requiring(flywheel, intake);
    }
    public static Command timedReverseIntake(double power, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(timer::reset)
                .setExecute(() -> {
                    if (timer.seconds() >= seconds) {
                        intake.reverse();
                    }
                })
                .setDone(() -> timer.seconds() >= seconds + 1)
                .setEnd(endCondition -> {
                    intake.setPower(0);
                })
                .requiring(flywheel, intake);
    }
}
