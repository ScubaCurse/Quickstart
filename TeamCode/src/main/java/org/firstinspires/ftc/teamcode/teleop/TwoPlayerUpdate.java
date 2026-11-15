package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utilities.RobotPosition;
import org.json.JSONException;
import org.json.JSONObject;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.math.Pose;
@TeleOp(name = "TwoPlayerUpdate")
public class TwoPlayerUpdate extends LinearOpMode {
    // Gamepad 1 is driver, gamepad 2 is intake/shooter
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

        speedMode = 0.01;

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

            if (gamepad2.right_bumper) {

                if (gamepad2.right_trigger > 0) {
                    shoot.shootWithIntake(targetVelocity);
                } else {
                    shoot.shoot(targetVelocity);
                }

            } else if (gamepad2.left_trigger > 0) {

                shoot.reverseShooter();

            } else if (gamepad2.left_bumper) {

                shoot.reverseIntake();
            } else {

                shoot.stop();

            }

            // SHOOTER VELOCITY ADJUSTMENT WITH DEBOUNCE
            if (gamepad2.dpad_up && !dpadUpPressed) {
                targetVelocity += 25;
                dpadUpPressed = true;
            }
            if (!gamepad2.dpad_up) {
                dpadUpPressed = false;
            }
            if (gamepad2.dpad_down && !dpadDownPressed) {
                targetVelocity -= 25;
                dpadDownPressed = true;
            }
            if (!gamepad2.dpad_down) {
                dpadDownPressed = false;
            }
            // rumble controller when shooter velocity is within 25 of the target.
            if (Math.abs(flywheel.getAverageVelocity() - targetVelocity) <= 50) {
                gamepad2.rumble(500);
            }

            // TELEMETRY
            telemetry.addData("TargetVelocity", targetVelocity);
            telemetry.addData("ShooterVelocity", flywheel.getAverageVelocity());
            // telemetry.addData("ShooterLV", flywheel.getLeftVelocity());
            // telemetry.addData("ShooterRV", flywheel.getRightVelocity());
            RobotPosition robotPosition = new RobotPosition();

            JSONObject savedPosition = robotPosition.load(hardwareMap);

            if (savedPosition != null) {

                double stoppedX = 0;
                try {
                    stoppedX = savedPosition.getDouble("stoppedX");
                } catch (JSONException e) {
                    throw new RuntimeException(e);
                }
                double stoppedY = 0;
                try {
                    stoppedY = savedPosition.getDouble("stoppedY");
                } catch (JSONException e) {
                    throw new RuntimeException(e);
                }
                double stoppedHeading = 0;
                try {
                    stoppedHeading = savedPosition.getDouble("stoppedHeading");
                } catch (JSONException e) {
                    throw new RuntimeException(e);
                }
                telemetry.addData("Robot stopped in auto", "");
                telemetry.addData("Saved X", stoppedX);
                telemetry.addData("Saved Y", stoppedY);
                telemetry.addData("Saved Heading", stoppedHeading);

            } else {
                telemetry.addData("Error", "Could not load robot position");
            }

            telemetry.update();
            telemetry.update();

            // DRIVE SPEED CONTROL
            if (gamepad2.x) {
                speedMode = 3;
            }
            if (gamepad2.y) {
                speedMode = 0.01;
            }
        }
    }
}
