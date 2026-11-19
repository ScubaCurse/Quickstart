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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Shoot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name = "ShootAndPark", group = "Autonomous")
public class ShootAndPark extends LinearOpMode {

    private Follower follower;
    private Shoot shoot;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.493, 7.8341, -90);
    private final Pose path1 = poseFactory.of(11.211, 117.6155, 0);
    private final Pose path1Control1 = poseFactory.of(10.2535, 30.6464, 0);

    public Path path1() {
        return Paths.curve(start, path1Control1, path1)
                .linear(start, path1);
    }

    public Command autoRoutine() {
        return sequential(
                shoot.shootForTime(1783.133333333, 2),
                follow(follower, path1())
        );
    }

    @Override
    public void runOpMode() {

        Scheduler.reset();

        // INITIALIZE HARDWARE
        follower = Constants.create(hardwareMap);

        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        shoot = new Shoot(flywheel, intake);

        // SET STARTING POSITION
        follower.setPose(start);

        // Stores the last valid robot position
        Pose lastValidPose = follower.pose();

        // SAVE INITIAL POSITION
        OpMode.blackboard.put("pose", lastValidPose);

        telemetry.addLine("ShootAndPark Initialized");
        telemetry.addData("Initial Pose", lastValidPose);
        telemetry.update();

        try {

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            schedule(autoRoutine());

            while (opModeIsActive()) {

                follower.update();

                Pose currentPose = follower.pose();

                // Had issues with odometry resetting itself to (0.0, 0.0, 0.0) when stopping.
                // It would save that empty value. This if statement prevents that.
                if (Math.abs(currentPose.x()) > 0.001 ||
                        Math.abs(currentPose.y()) > 0.001 ||
                        Math.abs(currentPose.heading()) > 0.001) {

                    lastValidPose = currentPose;

                    OpMode.blackboard.put("pose", lastValidPose);
                }

                // EXECUTE COMMANDS
                Scheduler.execute();

                // TELEMETRY
                telemetry.addLine("--- ROBOT POSITION ---");

                telemetry.addData("X", currentPose.x());
                telemetry.addData("Y", currentPose.y());
                telemetry.addData("Heading", currentPose.heading());

                telemetry.addLine("--- LAST VALID POSE ---");

                telemetry.addData("X", lastValidPose.x());
                telemetry.addData("Y", lastValidPose.y());
                telemetry.addData("Heading", lastValidPose.heading());

                telemetry.addLine("--- BLACKBOARD ---");

                telemetry.addData(
                        "Blackboard",
                        OpMode.blackboard.get("pose")
                );

                telemetry.addData(
                        "Pose Exists",
                        OpMode.blackboard.containsKey("pose")
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

        } finally {

            OpMode.blackboard.put("pose", lastValidPose);

            telemetry.addLine("--- FINAL SAVED POSE ---");

            telemetry.addData("X", lastValidPose.x());
            telemetry.addData("Y", lastValidPose.y());
            telemetry.addData("Heading", lastValidPose.heading());

            telemetry.update();
        }
    }
}
