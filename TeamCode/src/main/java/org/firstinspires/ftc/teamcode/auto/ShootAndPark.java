package org.firstinspires.ftc.teamcode.auto;

// Pedro Pathing imports
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

// OpMode imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Our own
import static org.firstinspires.ftc.teamcode.commands.Shoot.*;

@Autonomous(name = "ShootAndPark", group = "Autonomous")
public class ShootAndPark extends LinearOpMode {

    private Follower follower;
    private Flywheel flywheel;
    private Intake intake;
    private Shoot shoot;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.493, 7.8341, -90);
    private final Pose path1 = poseFactory.of(11.211, 117.6155, 0);
    private final Pose path1Control1 = poseFactory.of(10.2535, 30.6464, 0);

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    // Autonomous routine
    public Command autoRoutine() {

        return sequential(
                shoot.shootForTime(1725,5),
                follow(follower, path1())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);

        shoot = new Shoot(flywheel, intake);

        follower.setPose(start);
        follower.update();
        Command movePlusIntake = parallel(
                follow(follower, path1())
                //Shoot.timedIntake(1,5)
        );
        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();


            /*
            blackboard.put("poseX", follower.pose().x());
            blackboard.put("poseY", follower.pose().y());
            blackboard.put("poseHeading", follower.pose().heading());

            */
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
