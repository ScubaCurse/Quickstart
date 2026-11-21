
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Shoot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name = "ShootAndCollectTwice", group = "Autonomous")
public class ShootCollectShootPark extends LinearOpMode {

    private Shoot shoot;
    private Follower follower;

    public Pose lastValidPose;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.493, 7.8341, -90);
    private final Pose collectPollen1 = poseFactory.of(8.391, 6.6383, 180);
    private final Pose collectPollen1Control1 = poseFactory.of(58.8716, 34.2268, 0);
    private final Pose collectPollen1Control2 = poseFactory.of(51.1161, 16.0237, 0);
    private final Pose halfFlowerHive = poseFactory.of(25.3946, 45.7615, 180);
    private final Pose pathToHive = poseFactory.of(59.6191, 111.4372, 90);
    private final Pose pathToHiveControl1 = poseFactory.of(9.8816, 141.5, 0);
    private final Pose lineUpHive2 = poseFactory.of(60.9428, 133.462, 90);
    private final Pose parkStart = poseFactory.of(60.9428, 133.462, 90);
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


    // Commands

    public Command movePlusIntake() {
        return parallel(
                follow(follower, collectPollen1()),
                shoot.timedIntake(1, 5)
        );
    }

    public Command reverseIntakeHalf() {
        return parallel(
                follow(follower, halfFlowerHive()),
                shoot.timedReverseIntake(0.1, 0.1)
        );
    }

    public Command autoRoutine() {
        return sequential(
                shoot.shootForTime(1725, 2),

                movePlusIntake(),

                reverseIntakeHalf(),

                follow(follower, pathToHive()),

                follow(follower, lineUpHive2()),

                shoot.shootForTime(1725, 2),

                follow(follower, park())
        );
    }


    // Main OpMode

    @Override
    public void runOpMode() {

        // Reset scheduler
        Scheduler.reset();

        // Initialize subsystems
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        shoot = new Shoot(flywheel, intake);

        // Initialize follower
        follower = Constants.create(hardwareMap);

        // Set starting position
        follower.setPose(start);

        lastValidPose = start;

        follower.update();


        // Wait for start
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // Schedule autonomous routine
        schedule(autoRoutine());


        // Main loop
        while (opModeIsActive()) {

            // Update follower
            follower.update();

            // Get current pose
            Pose currentPose = follower.pose();

            // Prevent invalid odometry readings
            if (currentPose != null &&
                    (Math.abs(currentPose.x()) > 0.001 ||
                            Math.abs(currentPose.y()) > 0.001 ||
                            Math.abs(currentPose.heading()) > 0.001)) {

                lastValidPose = currentPose;

                OpMode.blackboard.put("pose", lastValidPose);
            }

            // Execute scheduler
            Scheduler.execute();

            // Update blackboard
            OpMode.blackboard.put("Blackboard", lastValidPose);


            // Telemetry
            telemetry.addData("X", currentPose.x());
            telemetry.addData("Y", currentPose.y());
            telemetry.addData("Heading", currentPose.heading());

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
