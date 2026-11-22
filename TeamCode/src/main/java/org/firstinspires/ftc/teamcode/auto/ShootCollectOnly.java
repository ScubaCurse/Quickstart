package org.firstinspires.ftc.teamcode.auto;

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
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.Shoot;

@Autonomous(name = "Shoot Collect Only", group = "Autonomous")
public class ShootCollectOnly extends LinearOpMode {

    private Follower follower;
    private Shoot shoot;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose lineUpCorner = poseFactory.of(7.7274, 17.7526, -90);
    private final Pose push1 = poseFactory.of(7.4047, 7.8024, -90);
    private final Pose pull1 = poseFactory.of(7.6497, 17.1758, -90);
    private final Pose push2 = poseFactory.of(7.5528, 7.4258, -90);
    private final Pose pull2 = poseFactory.of(7.9298, 17.2352, -90);
    private final Pose push3 = poseFactory.of(7.3101, 7.1782, -90);
    private final Pose point7 = poseFactory.of(10.2164, 93.3671, -90);
    private final Pose point7Control1 = poseFactory.of(48.3454, 41.6325, 0);

    // Autonomous routine

    public Command moveIntake() {
        return parallel(
                follow(follower, push1()),
                follow(follower, pull1()),
                follow(follower, push2()),
                follow(follower, pull2()),
                follow(follower, push3()),
                shoot.timedIntake(1, 10)
        );
    }
    public Command autoRoutine() {
        return sequential(
                shoot.shootForTime(1725,2),

                follow(follower, lineUpCorner()),

                follow(follower, path7())
        );
    }

    @Override
    public void runOpMode() {
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

    public Path lineUpCorner() {
        return Paths.line(start, lineUpCorner).constant(lineUpCorner);
    }

    public Path push1() {
        return Paths.line(lineUpCorner, push1).constant(push1);
    }

    public Path pull1() {
        return Paths.line(push1, pull1).constant(pull1);
    }

    public Path push2() {
        return Paths.line(pull1, push2).constant(push2);
    }

    public Path pull2() {
        return Paths.line(push2, pull2).constant(pull2);
    }

    public Path push3() {
        return Paths.line(pull2, push3).constant(push3);
    }

    public Path path7() {
        return Paths.curve(push3, point7Control1, point7).constant(point7);
    }
}
