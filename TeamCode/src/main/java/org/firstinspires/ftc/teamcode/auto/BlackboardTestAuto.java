package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Blackboard Test Auto")
public class BlackboardTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        if (isStopRequested()) return;

        OpMode.blackboard.put("testValue", "HELLO FROM AUTO");

        telemetry.addData("Written",
                OpMode.blackboard.get("testValue"));
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}