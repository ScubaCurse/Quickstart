package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret OpMode")
public class TurretOpMode extends OpMode {

    private Follower follower;
    private Flywheel flywheel;
    private Intake intake;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);

        if (blackboard.get("pose") == null) {
            telemetry.addLine("ERROR: Pose is null, did you run your auto?");
        }

        blackboard.putIfAbsent("pose", new Pose(0, 0, 0));

        // Restore pose from auto
        follower.setPose((Pose) blackboard.get("pose"));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Field-centric drivetrain
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x / 1.5,
                follower.pose().heading()
        );
        ManualDrive.driveOrHold(follower, powers);

        // relocalise button
        if (gamepad1.dpadLeftWasPressed()) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            // On the fly Pose creation, we don't recommend this for Autonomous. Only accepts radians for heading
            follower.setPose(cornerPose); // overrides our pose
        }

        // shooting
        if (gamepad1.right_bumper) {
            if (gamepad1.right_trigger > 0) {
                flywheel.setVelocity(1725);
                intake.setPower(0.5);
            } else {
                flywheel.setVelocity(1725);
                intake.stop();
            }
        } else if (gamepad1.left_trigger > 0) {
            flywheel.setVelocity(-500);
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
        telemetry.addData("Shooter Velocity", flywheel.getAverageVelocity());

        telemetry.addLine("--- ROBOT POSITION ---");
        telemetry.addData("Position", follower.pose());
        telemetry.addData("Blackboard Saved", blackboard.get("pose"));
        telemetry.update();
    }
}
