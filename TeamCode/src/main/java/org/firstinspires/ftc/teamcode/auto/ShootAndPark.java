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

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.Shoot;

import java.util.Objects;

@Autonomous(name = "Shoot and Park", group = "Autonomous")
public class ShootAndPark extends LinearOpMode {

    private Follower follower;
    private Shoot shoot;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(58.7637, 8, -90);
    private final Pose park = poseFactory.of(10.4915, 95.3268, 0);
    private final Pose parkControl1 = poseFactory.of(19.0665, 35.95, 0);
    public Path park() {
        return Paths.curve(start, parkControl1, park).linear(start, park);
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                shoot.shootForTime(1725,3),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        shoot = new Shoot(hardwareMap);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            // Poses resets itself every time the OpMode stops, so a check is required.
            if (follower.pose().x() != 0 && follower.pose().y() != 0 && follower.pose().heading() != 0) {
                blackboard.put("pose", follower.pose());
            }

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
}
