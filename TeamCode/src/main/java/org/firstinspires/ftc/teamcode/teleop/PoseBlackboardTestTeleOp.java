package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pose Blackboard Test TeleOp")
public class PoseBlackboardTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        if (isStopRequested()) return;

        Object value = OpMode.blackboard.get("pose");

        telemetry.addData("Retrieved Pose", value);
        telemetry.addData("Is Null", value == null);

        if (value instanceof Pose) {
            Pose pose = (Pose) value;

            telemetry.addData("X", pose.x());
            telemetry.addData("Y", pose.y());
            telemetry.addData("Heading", pose.heading());
        }

        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}