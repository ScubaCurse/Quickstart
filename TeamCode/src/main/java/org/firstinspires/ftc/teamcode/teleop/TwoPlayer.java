package org.firstinspires.ftc.teamcode.teleop;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.util.Calculator;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.HiveShotTableAudience;
import org.firstinspires.ftc.teamcode.util.HiveShotTableFar;
import org.firstinspires.ftc.teamcode.util.Shoot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

@TeleOp(name = "Two Player")
public class TwoPlayer extends LinearOpMode {

    // Gamepad 1 = Driver
    // Gamepad 2 = Intake / Shooter

    private double hiveX = 57.5;
    private double hiveY = 43.0;
    private int hiveSelected = 1;

    @Override
    public void runOpMode() {

        // INITIALIZE HARDWARE
        Follower follower = Constants.create(hardwareMap);
        //Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shoot shoot = new Shoot(flywheel, intake);
        double targetRPM = 0;

        // INITIALIZE PINPOINT
        /*
        PinpointLocalizer localizer = new PinpointLocalizer(
                hardwareMap,
                Constants.localizerConfig
        );
        */
        telemetry.addLine("TeleOp Initialized");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        if (OpMode.blackboard.get("pose") == null) {
            telemetry.addLine("ERROR: Pose is null, resetting to 0. Did you run your auto yet?");
        }
        OpMode.blackboard.putIfAbsent("pose", new Pose(0.0,0.0,0.0));

        // Restore Pinpoint starting position ONCE
        Object value = OpMode.blackboard.get("pose");
        Pose pose = (Pose) value;
        follower.setPose(pose);

        // MAIN TELEOP LOOP

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
            /*
            double angleToTarget = Math.atan2(
                    hiveY - robotY,
                    hiveX - robotX
            );
            double targetHeading = angleToTarget + Math.PI;
            double headingError = targetHeading - localizer.pose().heading();

            headingError = Math.atan2(
                    Math.sin(headingError),
                    Math.cos(headingError)
            );

            // Heading controller
            double kP = 1.5;

            double turnPower = Math.max(
                    -0.6,
                    Math.min(0.6, headingError * kP)
            );

             */

            if (hiveSelected == 1) {
                targetRPM = HiveShotTableAudience.rpm(robotX,robotY);
            } else if (hiveSelected == 2) {
                targetRPM = HiveShotTableFar.rpm(robotX,robotY);
            }

            // DISTANCE TO HIVE

            double distance = Math.sqrt(
                    Math.pow(hiveY - robotY, 2) +
                            Math.pow(hiveX - robotX, 2)
            );

            // DRIVETRAIN CONTROL

            /*
            double axial = -(gamepad1.left_stick_y / speedMode);
            double lateral = gamepad1.right_stick_x / 2;
            double yaw = gamepad1.left_stick_x / speedMode;

            drivetrain.mecanum(axial, lateral, yaw);
            */
            ManualDrive.driveOrHold(
                    follower,
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x/1.5
            );


            // SHOOTER CONTROL
            /*
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;

            double turn;

            if (gamepad1.a) {
                turn = turnPower;
            } else {
                turn = gamepad1.right_stick_x;
            }
            */
            if (gamepad2.right_bumper) {
                // aim towards selected hive
                /*
                Pose current = follower.pose();

                double heading = Math.atan2(
                        hiveY - current.y(),
                        hiveX - current.x()
                );

                // Face the target with the back (add 180 degrees)
                heading += Math.PI;

                follower.hold(new Pose(
                        current.x(),
                        current.y(),
                        heading
                ));
                */
                if (gamepad2.right_trigger > 0) {

                    flywheel.setVelocity(1725);
                    intake.intake(0.5);

                } else {

                    flywheel.setVelocity(1725);
                    intake.stop();

                }

            } else if (gamepad2.left_trigger > 0) {

                flywheel.reverse();
                intake.setPower(1);

            } else if (gamepad2.left_bumper) {

                flywheel.stop();
                intake.reverse();

            } else {

                shoot.stop();
            }

            // SHOOTER RUMBLE

            if (Math.abs(
                    flywheel.getAverageVelocity() - Calculator.rpmToTps(targetRPM, 28)
            ) <= 100) {

                gamepad2.rumble(500);
            }
            // TELEMETRY

            telemetry.addLine("--- SHOOTER ---");
            telemetry.addData(
                    "Shooter Velocity",
                    flywheel.getAverageVelocity()
            );
            telemetry.addData(
                    "Shooter RPM",
                    Calculator.tpsToRpm(
                            flywheel.getAverageVelocity(), 28
                    )
            );
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Target TPS", Calculator.rpmToTps(targetRPM, 28));

            telemetry.addLine("--- HIVE ---");
            telemetry.addData("Hive Selected", hiveSelected);
            telemetry.addData("Distance to Hive", distance);


            telemetry.addLine("--- ROBOT POSITION ---");
            telemetry.addData("Pos", follower.pose());
            telemetry.addData("Blackboard saved", pose);
            telemetry.update();

            // DRIVE SPEED CONTROL
            /*
            double speedMode = 0.001;
            if (gamepad1.dpad_up) {
                speedMode = 3;
            }

            if (gamepad1.dpad_down) {
                speedMode = 0.01;
            }
             */
        }

        // STOP MECHANISMS

        shoot.stop();
    }

}
