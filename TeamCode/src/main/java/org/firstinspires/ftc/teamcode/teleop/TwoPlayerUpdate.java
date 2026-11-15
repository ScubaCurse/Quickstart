package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ShootCalc;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utilities.RobotPosition;
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
    private double hiveX;
    private double hiveY;
    private int hiveSelected;
    private double distance;

    @Override
    public void runOpMode() {

        targetVelocity = 1725;

        // Hive 2 selected by default
        hiveX = 57.5;
        hiveY = 96.9;
        hiveSelected = 2;

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
        // Load saved autonomous position
        RobotPosition position = new RobotPosition();
        JSONObject saved = position.load(hardwareMap);

        // Initialize Pinpoint
        localizer = new PinpointLocalizer(
                hardwareMap,
                Constants.localizerConfig
        );

        // Restore field position
        if (saved != null) {
            double x = saved.optDouble("stoppedX", Double.NaN);
            double y = saved.optDouble("stoppedY", Double.NaN);
            double heading = saved.optDouble("stoppedHeading", Double.NaN);

            if (!Double.isNaN(x) &&
                    !Double.isNaN(y) &&
                    !Double.isNaN(heading)) {

                localizer.setPose(new Pose(x, y, heading));
            }
        }
        waitForStart();

        while (opModeIsActive()) {

            localizer.update();

            Pose currentPose = localizer.pose();

            double robotX = currentPose.x();
            double robotY = currentPose.y();
            double robotHeading = currentPose.heading();
            // Change hive selected
            if (gamepad2.xWasPressed()) {
                hiveSelected = 1;
                hiveX = 57.5;
                hiveY = 43.0;
            } else if (gamepad2.yWasPressed()) {
                hiveSelected = 2;
                hiveX = 57.5;
                hiveY = 96.9;
            }
            distance = Math.sqrt(((hiveY - robotY) * (hiveY - robotY)) + ((hiveX - robotX) * (hiveX - robotX)));

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

            // SHOOTER VELOCITY ADJUSTMENT
            if (gamepad2.dpadUpWasPressed()) {
                targetVelocity += 25;
            }
            if (gamepad2.dpadDownWasPressed()) {
                targetVelocity -= 25;
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

                double stoppedX = savedPosition.optDouble("stoppedX", Double.NaN);
                double stoppedY = savedPosition.optDouble("stoppedY", Double.NaN);
                double stoppedHeading = savedPosition.optDouble("stoppedHeading", Double.NaN);

                if (!Double.isNaN(stoppedX) &&
                        !Double.isNaN(stoppedY) &&
                        !Double.isNaN(stoppedHeading)) {

                    telemetry.addData("Saved X", stoppedX);
                    telemetry.addData("Saved Y", stoppedY);
                    telemetry.addData("Saved Heading", stoppedHeading);
                    telemetry.addData("Robot X", robotX);
                    telemetry.addData("Robot Y", robotY);
                    telemetry.addData("Robot Heading", robotHeading);

                } else {
                    telemetry.addData("Error", "Invalid or missing coordinates in JSON");
                }

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
