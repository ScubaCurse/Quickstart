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

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(13.3675, 102.354, 180);
    private final Pose path1Control1 = poseFactory.of(3.4522, 27.8263, 0);

    private Flywheel flywheel;
    private Intake intake;
    private Shoot shoot;

    private RobotPosition robotPosition;

    public Command autoRoutine() {
        return sequential(
                follow(follower, path1())
        );
    }

    @Override
    public void runOpMode() {

        // Initialize subsystems
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
        shoot = new Shoot(flywheel, intake);

        robotPosition = new RobotPosition();

        // Initialize Pedro
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        try {

            // Shoot preloaded POLLEN
            shoot.shootForTime(1725, 2.0, this);

            if (opModeIsActive()) {

                // Start autonomous paths
                schedule(autoRoutine());

                while (opModeIsActive()) {

                    follower.update();
                    Scheduler.execute();

                    // Save the latest robot position
                    Pose currentPose = follower.pose();

                    telemetry.addData("x", currentPose.x());
                    telemetry.addData("y", currentPose.y());
                    telemetry.addData("heading", currentPose.heading());

                    if (follower.currentPath() != null) {
                        telemetry.addData(
                                "Current path distance remaining",
                                follower.distanceToEndpoint()
                        );

                        telemetry.addData(
                                "Path number",
                                follower.pathIndex()
                        );
                    }

                    telemetry.update();
                }
            }

        } finally {

            // Save the last known position when autonomous ends
            if (follower != null) {

                Pose finalPose = follower.pose();

                robotPosition.save(
                        hardwareMap,
                        finalPose.x(),
                        finalPose.y(),
                        finalPose.heading()
                );
            }

            // Stop mechanisms
            shoot.stop();
        }
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1)
                .linear(start, path1);
    }
}
/*
package org.firstinspires.ftc.teamcode.auto;
n
import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.RobotPosition;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(13.3675, 102.354, 180);
    private final Pose path1Control1 = poseFactory.of(3.4522, 27.8263, 0);

    private Flywheel flywheel;
    private Intake intake;
    private Shoot shoot;
    private RobotPosition robotPosition;
    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1())
        );
    }

    @Override
    public void runOpMode() {
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shoot shoot = new Shoot(flywheel, intake);
        RobotPosition robotPosition = new RobotPosition();
        robotPosition.save(hardwareMap, follower.pose().x(), follower.pose().y(), follower.pose().heading());
        // Shoot preloaded pollen
        shoot.shoAnnythingotForTime(1725, 2.0, this);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }
}
*/