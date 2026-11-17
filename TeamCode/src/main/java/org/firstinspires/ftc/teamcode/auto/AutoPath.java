
package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.RobotPosition;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(
            61.1424, 8.6635, -90
    );

    private final Pose path1 = poseFactory.of(
            9.5522, 110.4824, 180
    );

    private final Pose path1Control1 = poseFactory.of(
            16.5571, 19.5321, 0
    );

    private Flywheel flywheel;
    private Intake intake;
    private Shoot shoot;

    private Pose lastPose;

    // Save every 100 milliseconds
    private static final long SAVE_INTERVAL = 100;

    public Command autoRoutine() {
        return sequential(
                follow(follower, path1())
        );
    }

    @Override
    public void runOpMode() {

        try {

            // Initialize subsystems
            flywheel = new Flywheel(hardwareMap);
            intake = new Intake(hardwareMap);
            shoot = new Shoot(flywheel, intake);

            Scheduler.reset();

            // Initialize Pedro
            follower = Constants.create(hardwareMap);

            follower.setPose(start);
            follower.update();

            lastPose = follower.pose();

            // Save starting position
            savePosition();

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Starting X", lastPose.x());
            telemetry.addData("Starting Y", lastPose.y());
            telemetry.addData("Starting Heading", lastPose.heading());
            telemetry.update();

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            if (opModeIsActive()) {

                // Shoot preloaded pollen
                Shoot.shootForTime(1700, 2.0);

                if (opModeIsActive()) {

                    schedule(autoRoutine());

                    long lastSaveTime = 0;

                    // MAIN AUTONOMOUS LOOP
                    while (opModeIsActive()) {

                        // Update Pedro localization
                        follower.update();

                        // Get latest position
                        lastPose = follower.pose();

                        // Save position every 100 ms
                        long currentTime = System.currentTimeMillis();

                        if (currentTime - lastSaveTime >= SAVE_INTERVAL) {

                            savePosition();

                            lastSaveTime = currentTime;
                        }
                        
                        // Execute scheduled commands
                        Scheduler.execute();

                        // Telemetry
                        telemetry.addData("LIVE X", lastPose.x());
                        telemetry.addData("LIVE Y", lastPose.y());
                        telemetry.addData("LIVE Heading", lastPose.heading());

                        telemetry.addData(
                                "Last Save",
                                currentTime - lastSaveTime + SAVE_INTERVAL + " ms"
                        );

                        if (follower.currentPath() != null) {

                            telemetry.addData(
                                    "Distance Remaining",
                                    follower.distanceToEndpoint()
                            );

                            telemetry.addData(
                                    "Path Number",
                                    follower.pathIndex()
                            );
                        }

                        telemetry.update();
                    }
                }
            }

        } finally {

            // Do NOT save position here.
            // The last successful periodic save remains in JSON.

            // Stop mechanisms
            if (shoot != null) {
                shoot.stop();
            }
        }
    }

    private void savePosition() {

        if (lastPose == null) {
            return;
        }

        double x = lastPose.x();
        double y = lastPose.y();
        double heading = lastPose.heading();

        if (!Double.isFinite(x) ||
                !Double.isFinite(y) ||
                !Double.isFinite(heading)) {

            android.util.Log.e(
                    "AutoPath",
                    "Invalid pose. Skipping save."
            );

            return;
        }

        RobotPosition.save(
                hardwareMap,
                x,
                y,
                heading
        );
    }

    public Path path1() {

        return Paths.curve(start, path1Control1, path1)
                .linear(start, path1);
    }
}
