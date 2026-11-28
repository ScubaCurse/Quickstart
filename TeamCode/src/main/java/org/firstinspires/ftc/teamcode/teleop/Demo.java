package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Demo")
public class Demo extends LinearOpMode {
    // Gamepad 1 = Driver
    // Gamepad 2 = Intake / Shooter
    private double hiveX = 57.5;
    private double hiveY = 43.0;
    private int hiveSelected = 1;

    @Override
    public void runOpMode() {

        Follower follower = Constants.create(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        boolean drivingHold = true;
        telemetry.addLine("TeleOp Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        follower.setPose(new Pose(8.3, 8.0, Math.toRadians(90)));

        while (opModeIsActive()) {

            follower.update();
            double robotX = follower.pose().x();
            double robotY = follower.pose().y();
            double robotHeading = follower.pose().heading();

            // relocalise button
            if (gamepad1.dpadLeftWasPressed()) {
                Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
                // On the fly Pose creation, we don't recommend this for Autonomous. Only accepts radians for heading
                follower.setPose(cornerPose); // overrides our pose
            }

            // Select hives
            if (gamepad2.xWasPressed()) {

                hiveSelected = 1;
                hiveX = 57.5;
                hiveY = 43.0;

            } else if (gamepad2.yWasPressed()) {

                hiveSelected = 2;
                hiveX = 57.5;
                hiveY = 96.9;
            }

            // Hive distance
            double distance = Math.sqrt(
                    Math.pow(hiveY - robotY, 2) +
                            Math.pow(hiveX - robotX, 2)
            );

            /*
            Pose bottomLeft = new Pose(8.3, 8.0, 0.0);
            Pose bottomRight = new Pose(62.2, 8.0, 0.0);
            Pose topLeft = new Pose(8.3, 133.2, 0.0);
            Pose topRight  = new Pose(62.2, 133.2, 0.0);
            */

            if (gamepad1.x) {
                drivingHold = false;

            } else if (gamepad1.y) {
                drivingHold = true;
            }

            double x = follower.pose().x();
            double y = follower.pose().y();

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            // X boundaries
            if (x <= 20 && forward < 0) {
                forward = 0;
            }
            if (x >= 120 && forward > 0) {
                forward = 0;
            }

            // Y boundaries
            if (y <= 20 && strafe < 0) {
                strafe = 0;
            }
            if (y >= 120 && strafe > 0) {
                strafe = 0;
            }

            DrivePowers powers = ManualDrive.fieldCentric(
                    forward,
                    strafe,
                    turn,
                    follower.pose().heading()
            );

            if (drivingHold) {
                ManualDrive.driveOrHold(follower, powers);
            } else {
                follower.manual(powers);
            }

            follower.update();


            // shooting
            if (gamepad1.right_bumper) {

                if (gamepad1.right_trigger > 0) {

                    flywheel.setVelocity(2566);
                    intake.setPower(1);

                } else {

                    flywheel.setVelocity(2566);
                    intake.stop();

                }

            } else if (gamepad1.left_trigger > 0) {

                flywheel.setVelocity(-1000);
                intake.setPower(1);

            } else if (gamepad1.left_bumper) {

                flywheel.stop();
                intake.setPower(-0.3);

            } else {

                flywheel.stop();
                intake.stop();
            }

            // Telemetry
            telemetry.addLine("--- SHOOTER ---");
            telemetry.addData(
                    "Shooter Velocity",
                    flywheel.getAverageVelocity()
            );

            telemetry.addLine("--- HIVE ---");
            telemetry.addData("Hive Selected", hiveSelected);
            telemetry.addData("Distance to Hive", distance);

            telemetry.addLine("--- ROBOT POSITION ---");
            telemetry.addData("Position", follower.pose());
            telemetry.update();
        }
    }
}
