package org.firstinspires.ftc.teamcode.auto;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Shoot;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "ShootCollectShootUpdate", group = "Autonomous")
public class ShootCollectShootUpdate extends LinearOpMode {

    private Shoot shoot;
    private Follower follower;
    public Pose lastValidPose;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose lineUpCorner = poseFactory.of(7.7274, 17.7526, -90);
    private final Pose push1 = poseFactory.of(7.4047, 7.8024, -90);
    private final Pose pull1 = poseFactory.of(7.6497, 17.1758, -90);
    private final Pose push2 = poseFactory.of(7.5528, 7.4258, -90);
    private final Pose pull2 = poseFactory.of(7.9298, 17.2352, -90);
    private final Pose push3 = poseFactory.of(7.3101, 7.1782, -90);
    private final Pose pull3Start = poseFactory.of(7.3101, 7.1782, -90);
    private final Pose pull3 = poseFactory.of(61.2331, 129.4463, 90);
    private final Pose pull3Control1 = poseFactory.of(42.8923, 91.8427, 0);
    private final Pose pull3Control2 = poseFactory.of(0.8307, 72.4109, 0);
    private final Pose lineUpHive2 = poseFactory.of(60.5382, 133.838, 90);
    private final Pose parkStart = poseFactory.of(60.5382, 133.838, 90);
    private final Pose park = poseFactory.of(10.1973, 123.2516, 0);
    private final Pose parkControl1 = poseFactory.of(57.5496, 101.6178, 0);

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

    public Path pull3() {
        return Paths.curve(pull3Start, pull3Control1, pull3Control2, pull3).linear(pull3Start, pull3);
    }

    public Path lineUpHive2() {
        return Paths.line(pull3, lineUpHive2).constant(lineUpHive2);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }

    // Commands

    public Command movePlusIntake() {
        return parallel(
                follow(follower, push1()),
                follow(follower, pull1()),
                follow(follower, push2()),
                follow(follower, pull2()),
                follow(follower, push3()),
                follow(follower, pull3()),
                shoot.timedIntake(1, 5)
        );
    }

    public Command moveReverseIntake() {
        return parallel(
                follow(follower, lineUpHive2()),
                shoot.timedReverseIntake(0.5, 0.5)
        );
    }
    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                shoot.shootForTime(1725,2),
                follow(follower, lineUpCorner()),
                movePlusIntake(),
                moveReverseIntake(),
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
