package org.firstinspires.ftc.teamcode.auto;

// Pedro Pathing imports

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

// OpMode imports
// Our own

@Autonomous(name = "ShootAndCollectTwice", group = "Autonomous")
public class ShootCollectShootPark extends LinearOpMode {

    private Flywheel flywheel;
    private Intake intake;
    private Shoot shoot;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.493, 7.8341, -90);
    private final Pose collectPollen1 = poseFactory.of(8.391, 8.1313, 180);
    private final Pose collectPollen1Control1 = poseFactory.of(58.8716, 34.2268, 0);
    private final Pose collectPollen1Control2 = poseFactory.of(51.1161, 16.0237, 0);
    private final Pose halfFlowerHive = poseFactory.of(25.3946, 45.7615, 180);
    private final Pose pathToHive = poseFactory.of(59.6191, 111.4372, 90);
    private final Pose pathToHiveControl1 = poseFactory.of(9.8816, 141.5, 0);
    private final Pose lineUpHive2 = poseFactory.of(60.9428, 133.462, 90);
    private final Pose collectFromHumanStart = poseFactory.of(60.9428, 133.462, 90);
    private final Pose collectFromHuman = poseFactory.of(9.2987, 110.6561, 180);
    private final Pose collectFromHumanControl1 = poseFactory.of(48.67, 104.5056, 0);
    private final Pose backToHive2Start = poseFactory.of(9.2987, 110.6561, 180);
    private final Pose backToHive2 = poseFactory.of(60.5636, 133.2824, 90);
    private final Pose backToHive2Control1 = poseFactory.of(48.8162, 104.6133, 0);
    private final Pose parkStart = poseFactory.of(60.5636, 133.2824, 90);
    private final Pose park = poseFactory.of(9.4741, 110.6735, 180);
    private final Pose parkControl1 = poseFactory.of(48.9452, 104.9789, 0);

    public Path collectPollen1() {
        return Paths.curve(start, collectPollen1Control1, collectPollen1Control2, collectPollen1).linear(start, collectPollen1);
    }

    public Path halfFlowerHive() {
        return Paths.line(collectPollen1, halfFlowerHive).constant(halfFlowerHive);
    }

    public Path pathToHive() {
        return Paths.curve(halfFlowerHive, pathToHiveControl1, pathToHive).constant(pathToHive);
    }

    public Path lineUpHive2() {
        return Paths.line(pathToHive, lineUpHive2).constant(lineUpHive2);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                shoot.shootForTime(1725,2),
                movePlusIntake,
                reverseIntakeHalf,
                follow(follower, pathToHive()),
                follow(follower, lineUpHive2()),
                shoot.shootForTime(1725,2),
                follow(follower, park())
        );
    }
    Command movePlusIntake = parallel(
            follow(follower, collectPollen1()),
            shoot.timedIntake(1,5)
    );
    Command reverseIntakeHalf = parallel(
            follow(follower, halfFlowerHive()),
            shoot.timedReverseIntake(0.1, 0.1)
    );
    @Override
    public void runOpMode() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);

        shoot = new Shoot(flywheel, intake);

        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            // My own code:
            blackboard.put("poseX", follower.pose().x());
            blackboard.put("poseY", follower.pose().y());
            blackboard.put("poseHeading", follower.pose().heading());
            // -

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
