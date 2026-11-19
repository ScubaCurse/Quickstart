package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Blackboard Test TeleOp")
public class BlackboardTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        if (isStopRequested()) return;

        Object value = OpMode.blackboard.get("testValue");

        telemetry.addData("Retrieved", value);
        telemetry.addData("Contains Key",
                OpMode.blackboard.containsKey("testValue"));
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}