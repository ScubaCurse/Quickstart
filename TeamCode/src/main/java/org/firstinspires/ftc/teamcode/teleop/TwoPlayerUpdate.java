package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.commands.Calculator;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TwoPlayerUpdate")
public class TwoPlayerUpdate extends LinearOpMode {

    // Gamepad 1 = Driver
    // Gamepad 2 = Intake / Shooter

    private double speedMode = 0.01;

    private double targetRPM = 0;

    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    private double hiveX = 57.5;
    private double hiveY = 96.9;
    private int hiveSelected = 2;

    @Override
    public void runOpMode() {

        // INITIALIZE HARDWARE
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Shoot shoot = new Shoot(flywheel, intake);

        // INITIALIZE PINPOINT

        PinpointLocalizer localizer = new PinpointLocalizer(
                hardwareMap,
                Constants.localizerConfig
        );

        telemetry.addLine("TeleOp Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // Restore Pinpoint starting position ONCE
        Object value = OpMode.blackboard.get("pose");
        Pose pose = (Pose) value;
        localizer.setPose(pose);

        // MAIN TELEOP LOOP

        while (opModeIsActive()) {

            // UPDATE LOCALIZATION

            localizer.update();

            double robotX = localizer.pose().x();
            double robotY = localizer.pose().y();
            double robotHeading = localizer.pose().heading();

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

            // DISTANCE TO HIVE

            double distance = Math.sqrt(
                    Math.pow(hiveY - robotY, 2) +
                            Math.pow(hiveX - robotX, 2)
            );

            // DRIVETRAIN CONTROL

            double axial = -(gamepad1.left_stick_y / speedMode);
            double lateral = gamepad1.right_stick_x / 2;
            double yaw = gamepad1.left_stick_x / speedMode;

            drivetrain.mecanum(axial, lateral, yaw);

            // SHOOTER CONTROL

            if (gamepad2.right_bumper) {

                if (gamepad2.right_trigger > 0) {

                    shoot.shootWithIntake(
                            Calculator.getTargetVelocity(distance)
                    );

                } else {

                    shoot.shoot(
                            Calculator.getTargetVelocity(distance)
                    );
                }

            } else if (gamepad2.left_trigger > 0) {

                shoot.reverseShooter();

            } else if (gamepad2.left_bumper) {

                shoot.reverseIntake();

            } else {

                shoot.stop();
            }

            // SHOOTER RUMBLE

            double targetVelocity = 1725;
            if (Math.abs(
                    flywheel.getAverageVelocity() - targetVelocity
            ) <= 100) {

                gamepad2.rumble(500);
            }

            // TARGET RPM ADJUSTMENT

            if (gamepad2.dpad_left && !dpadLeftPressed) {

                targetRPM += 250;

            } else if (gamepad2.dpad_right && !dpadRightPressed) {

                targetRPM -= 250;
            }

            dpadLeftPressed = gamepad2.dpad_left;
            dpadRightPressed = gamepad2.dpad_right;

            // TELEMETRY

            telemetry.addLine("--- SHOOTER ---");

            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData(
                    "Shooter Velocity",
                    flywheel.getAverageVelocity()
            );

            telemetry.addData("Target RPM", targetRPM);

            telemetry.addData(
                    "Shooter RPM",
                    Calculator.tpsToRpm(
                            flywheel.getAverageVelocity(), 28
                    )
            );

            telemetry.addLine("--- HIVE ---");

            telemetry.addData("Hive Selected", hiveSelected);
            telemetry.addData("Distance to Hive", distance);

            telemetry.addLine("--- LIVE ROBOT POSITION ---");

            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Robot Heading", robotHeading);

            telemetry.addLine("--- AUTONOMOUS POSITION ---");

            telemetry.addData("Blackboard", pose);

            telemetry.update();

            // DRIVE SPEED CONTROL

            if (gamepad1.dpad_up) {
                speedMode = 3;
            }

            if (gamepad1.dpad_down) {
                speedMode = 0.01;
            }
        }

        // STOP MECHANISMS

        shoot.stop();
    }
}
