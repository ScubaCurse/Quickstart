package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.math.Pose;
@TeleOp(name = "OnePlayerUpdate")
public class OnePlayerUpdate extends LinearOpMode {

    private Drivetrain drivetrain;
    private Flywheel flywheel;
    private Intake intake;
    private Shoot shoot;
    private Follower follower;
    private PinpointLocalizer localizer;
    private double speedMode;
    private int targetVelocity;

    // Button debounce variables
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void runOpMode() {

        targetVelocity = 1725;

        // Initialize subsystems
        drivetrain = new Drivetrain(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
        localizer = new PinpointLocalizer(
                hardwareMap,
                Constants.localizerConfig
        );
        follower.update();
        // Initialize commands
        shoot = new Shoot(flywheel, intake);

        speedMode = 0.001;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // DRIVETRAIN CONTROL

            double axial = -(gamepad1.left_stick_y / speedMode);
            double lateral = gamepad1.right_stick_x / 2;
            double yaw = gamepad1.left_stick_x / speedMode;

            drivetrain.mecanum(axial, lateral, yaw);


            // SHOOTER CONTROL

            if (gamepad1.right_bumper) {

                if (gamepad1.right_trigger > 0) {
                    shoot.shootWithIntake(targetVelocity);
                } else {
                    shoot.shoot(targetVelocity);
                }

            } else if (gamepad1.left_trigger > 0) {

                shoot.reverseShooter();

            } else if (gamepad1.left_bumper) {

                shoot.reverseIntake();
            } else {

                shoot.stop();

            }

            // SHOOTER VELOCITY ADJUSTMENT WITH DEBOUNCE
            if (gamepad1.dpad_up && !dpadUpPressed) {
                targetVelocity += 25;
                dpadUpPressed = true;
            }
            if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }
            if (gamepad1.dpad_down && !dpadDownPressed) {
                targetVelocity -= 25;
                dpadDownPressed = true;
            }
            if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }
            // rumble controller when shooter velocity is within 25 of the target.
            if (Math.abs(flywheel.getAverageVelocity() - targetVelocity) <= 50) {
                gamepad1.rumble(500);
            }

            // TELEMETRY
            telemetry.addData("TargetVelocity", targetVelocity);
            telemetry.addData("ShooterVelocity", flywheel.getAverageVelocity());
            // telemetry.addData("ShooterLV", flywheel.getLeftVelocity());
            // telemetry.addData("ShooterRV", flywheel.getRightVelocity());
            telemetry.update();

            // DRIVE SPEED CONTROL
            if (gamepad1.x) {
                speedMode = 3;
            }
            if (gamepad1.y) {
                speedMode = 0.001;
            }
        }
    }
}
