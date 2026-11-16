
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
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

@TeleOp(name = "OnePlayerUpdate")
public class OnePlayerUpdate extends LinearOpMode {

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

        speedMode = 0.01;

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

            if (gamepad1.xWasPressed()) {

                hiveSelected = 1;
                hiveX = 57.5;
                hiveY = 43.0;

            } else if (gamepad1.yWasPressed()) {

                hiveSelected = 2;
                hiveX = 57.5;
                hiveY = 96.9;
            }

            // Calculate distance to hive
            distance = Math.sqrt(
                    ((hiveY - robotY) * (hiveY - robotY)) +
                            ((hiveX - robotX) * (hiveX - robotX))
            );

            // DRIVETRAIN CONTROL

            double axial = -(gamepad1.left_stick_y / speedMode);
            double lateral = gamepad1.right_stick_x / 2;
            double yaw = gamepad1.left_stick_x / speedMode;

            drivetrain.mecanum(axial, lateral, yaw);


            // SHOOTER CONTROL

            if (gamepad1.right_bumper) {

                if (gamepad1.right_trigger > 0) {

                    shoot.shootWithIntake(
                            ShootCalc.getTargetVelocity(distance)
                    );

                } else {

                    shoot.shoot(
                            ShootCalc.getTargetVelocity(distance)
                    );
                }

            } else if (gamepad1.left_trigger > 0) {

                shoot.reverseShooter();

            } else if (gamepad1.left_bumper) {

                shoot.reverseIntake();

            } else {

                shoot.stop();
            }


            // Rumble when shooter velocity is close to target
            if (Math.abs(
                    flywheel.getAverageVelocity() - targetVelocity
            ) <= 100) {

                gamepad1.rumble(500);
            }


            // TELEMETRY

            telemetry.addData("Target Velocity", ShootCalc.getTargetVelocity(distance));
            telemetry.addData(
                    "Shooter Velocity",
                    flywheel.getAverageVelocity()
            );

            telemetry.addData("Hive Selected", hiveSelected);
            telemetry.addData("Distance to Hive", distance);

            telemetry.addLine("--- LIVE ROBOT POSITION ---");

            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Robot Heading", robotHeading);

            telemetry.addLine("--- SAVED AUTO POSITION ---");

            telemetry.addData("Saved X", savedX);
            telemetry.addData("Saved Y", savedY);
            telemetry.addData("Saved Heading", savedHeading);

            telemetry.update();


            // DRIVE SPEED CONTROL

            if (gamepad1.dpad_up) {
                speedMode = 3;
            }

            if (gamepad1.dpad_down) {
                speedMode = 0.01;
            }
        }

        // Stop mechanisms when TeleOp ends
        shoot.stop();
    }
}
