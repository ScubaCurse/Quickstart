package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriverControls;
import org.firstinspires.ftc.teamcode.commands.HiveSelector;
import org.firstinspires.ftc.teamcode.commands.ShooterControls;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.utilities.RobotTelemetry;

@TeleOp(name = "TwoPlayerUpdate")
public class TwoPlayerThree extends LinearOpMode {

    private Drivetrain drivetrain;
    private Flywheel flywheel;
    private Intake intake;

    private PinpointLocalizer localizer;

    private DriverControls driverControls;
    private ShooterControls shooterControls;
    private HiveSelector hiveSelector;
    private RobotTelemetry robotTelemetry;

    @Override
    public void runOpMode() {

        // Initialize hardware
        drivetrain = new Drivetrain(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);

        // Initialize commands
        Shoot shoot = new Shoot(flywheel, intake);

        // Initialize controls
        driverControls = new DriverControls(drivetrain);
        shooterControls = new ShooterControls(shoot);
        hiveSelector = new HiveSelector();

        // Initialize telemetry
        robotTelemetry = new RobotTelemetry(telemetry, flywheel);

        // Initialize Pinpoint
        localizer = new PinpointLocalizer(
                hardwareMap,
                Constants.localizerConfig
        );

        blackboard.putIfAbsent("poseX", 0);
        blackboard.putIfAbsent("poseY", 0);
        blackboard.putIfAbsent("poseHeading", 0);

        localizer.setPose(new Pose(
                (Double) blackboard.get("poseX"),
                (Double) blackboard.get("poseY"),
                (Double) blackboard.get("poseHeading")
        ));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Update localization
            localizer.update();

            Pose pose = localizer.pose();

            double robotX = pose.x();
            double robotY = pose.y();
            double robotHeading = pose.heading();

            // Update hive selection
            hiveSelector.update(gamepad2);

            double distance = hiveSelector.getDistance(
                    robotX,
                    robotY
            );

            // Update driver controls
            driverControls.update(gamepad1);

            // Update shooter controls
            shooterControls.update(gamepad2, distance);

            // Update telemetry
            robotTelemetry.update(
                    shooterControls.getTargetVelocity(),
                    shooterControls.getTargetRPM(),
                    hiveSelector.getSelectedHive(),
                    distance,
                    robotX,
                    robotY,
                    robotHeading
            );
        }

        // Stop mechanisms
        shooterControls.stop();
    }
}