
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.commands.ShootCalc;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utilities.RobotPosition;

import org.json.JSONObject;

@TeleOp(name = "OnePlayerOdo")
public class OnePlayerOdo extends LinearOpMode {

    // Gamepad 1 = Driver
    // Gamepad 2 = Intake / Shooter

    private Drivetrain drivetrain;
    private Flywheel flywheel;
    private Intake intake;
    private Shoot shoot;

    private PinpointLocalizer localizer;

    private double speedMode;

    private int targetVelocity;

    private double hiveX;
    private double hiveY;
    private int hiveSelected;

    private double distance;

    // DRIVE SETTINGS

    private static final double NORMAL_SPEED = 0.50;
    private static final double PRECISION_SPEED = 0.25;

    // Restored autonomous position
    private double savedX = Double.NaN;
    private double savedY = Double.NaN;
    private double savedHeading = Double.NaN;

    @Override
    public void runOpMode() {

        targetVelocity = 1725;

        // Hive 2 selected by default
        hiveX = 57.5;
        hiveY = 96.9;
        hiveSelected = 2;

        speedMode = NORMAL_SPEED;

        // Initialize subsystems
        drivetrain = new Drivetrain(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);

        shoot = new Shoot(flywheel, intake);

        // Initialize Pinpoint
        localizer = new PinpointLocalizer(
                hardwareMap,
                Constants.localizerConfig
        );

        // Load autonomous position ONCE
        JSONObject saved = RobotPosition.load(hardwareMap);

        if (saved != null) {

            savedX = saved.optDouble("stoppedX", Double.NaN);
            savedY = saved.optDouble("stoppedY", Double.NaN);
            savedHeading = saved.optDouble("stoppedHeading", Double.NaN);

            if (Double.isFinite(savedX) &&
                    Double.isFinite(savedY) &&
                    Double.isFinite(savedHeading)) {

                // Restore field position
                localizer.setPose(
                        new Pose(savedX, savedY, savedHeading)
                );

                telemetry.addData("Position", "Restored successfully");
                telemetry.addData("Saved X", savedX);
                telemetry.addData("Saved Y", savedY);
                telemetry.addData("Saved Heading", savedHeading);

            } else {

                telemetry.addData("Position", "Invalid saved coordinates");
            }

        } else {

            telemetry.addData("Position", "No saved position found");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

            // Update Pinpoint localization
            localizer.update();

            Pose currentPose = localizer.pose();

            double robotX = currentPose.x();
            double robotY = currentPose.y();
            double robotHeading = currentPose.heading();

            // HIVE SELECTION

            if (gamepad2.xWasPressed()) {

                hiveSelected = 1;
                hiveX = 57.5;
                hiveY = 43.0;

            } else if (gamepad2.yWasPressed()) {

                hiveSelected = 2;
                hiveX = 57.5;
                hiveY = 96.9;
            }

            // Calculate distance to hive
            distance = Math.sqrt(
                    ((hiveY - robotY) * (hiveY - robotY)) +
                            ((hiveX - robotX) * (hiveX - robotX))
            );

            // =====================================
            // DRIVETRAIN CONTROL
            // FIELD-CENTRIC
            // =====================================

            // Read joystick inputs
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.right_stick_x;
            double yaw = gamepad1.left_stick_x;

            // Mild joystick curve
            axial = smoothInput(axial);
            lateral = smoothInput(lateral);
            yaw = smoothInput(yaw);

            // Apply speed limit
            axial *= speedMode;
            lateral *= speedMode;
            yaw *= speedMode;

            // Convert field-centric inputs to robot-centric inputs
            double robotAxial =
                    axial * Math.cos(robotHeading)
                            + lateral * Math.sin(robotHeading);

            double robotLateral =
                    -axial * Math.sin(robotHeading)
                            + lateral * Math.cos(robotHeading);

            // Send to drivetrain
            drivetrain.mecanum(
                    robotAxial,
                    robotLateral,
                    yaw
            );

            // =====================================
            // SHOOTER CONTROL
            // =====================================

            if (gamepad2.right_bumper) {

                if (gamepad2.right_trigger > 0) {

                    shoot.shootWithIntake(
                            ShootCalc.getTargetVelocity(distance)
                    );

                } else {

                    shoot.shoot(
                            ShootCalc.getTargetVelocity(distance)
                    );
                }

            } else if (gamepad2.left_trigger > 0) {

                shoot.reverseShooter();

            } else if (gamepad2.left_bumper) {

                shoot.reverseIntake();

            } else {

                shoot.stop();
            }

            // Rumble when shooter velocity is close to target
            if (Math.abs(
                    flywheel.getAverageVelocity() - targetVelocity
            ) <= 100) {

                gamepad2.rumble(500);
            }

            // =====================================
            // TELEMETRY
            // =====================================

            telemetry.addData(
                    "Target Velocity",
                    ShootCalc.getTargetVelocity(distance)
            );

            telemetry.addData(
                    "Shooter Velocity",
                    flywheel.getAverageVelocity()
            );

            telemetry.addData("Hive Selected", hiveSelected);
            telemetry.addData("Distance to Hive", distance);

            telemetry.addLine("--- DRIVE SETTINGS ---");

            telemetry.addData("Speed Mode", speedMode);
            telemetry.addData("Drive Mode", "Field Centric");

            telemetry.addLine("--- LIVE ROBOT POSITION ---");

            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Robot Heading", robotHeading);

            telemetry.addLine("--- SAVED AUTO POSITION ---");

            telemetry.addData("Saved X", savedX);
            telemetry.addData("Saved Y", savedY);
            telemetry.addData("Saved Heading", savedHeading);

            telemetry.update();

            // =====================================
            // DRIVE SPEED CONTROL
            // =====================================

            if (gamepad1.dpad_up) {
                speedMode = NORMAL_SPEED;
            }

            if (gamepad1.dpad_down) {
                speedMode = PRECISION_SPEED;
            }
        }

        // Stop mechanisms when TeleOp ends
        drivetrain.stop();
        shoot.stop();
    }

    // =====================================
    // DRIVE HELPER
    // =====================================

    private double smoothInput(double input) {

        return 0.80 * input * input * input
                + 0.20 * input;
    }
}
