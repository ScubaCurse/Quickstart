package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Pose Blackboard Test Auto")
public class PoseBlackboardTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        if (isStopRequested()) return;

        Pose testPose = new Pose(57.493, 7.8341, -Math.PI / 2);

        OpMode.blackboard.put("pose", testPose);

        telemetry.addData("Saved Pose",
                OpMode.blackboard.get("pose"));
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}