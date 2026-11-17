package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.commands.Calculator;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

public class RobotTelemetry {

    private final org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    private final Flywheel flywheel;

    public RobotTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry, Flywheel flywheel) {
        this.telemetry = telemetry;
        this.flywheel = flywheel;
    }

    public void update(
            double targetVelocity,
            double targetRPM,
            int hiveSelected,
            double distance,
            double robotX,
            double robotY,
            double robotHeading
    ) {

        telemetry.addData("Target Velocity", targetVelocity);

        telemetry.addData(
                "Shooter Velocity",
                flywheel.getAverageVelocity()
        );

        telemetry.addData("Target RPM", targetRPM);

        telemetry.addData(
                "Shooter RPM",
                Calculator.tpsToRpm(
                        flywheel.getAverageVelocity(), 28
                )
        );

        telemetry.addData("Hive Selected", hiveSelected);
        telemetry.addData("Distance to Hive", distance);

        telemetry.addLine("--- LIVE ROBOT POSITION ---");
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Robot Heading", robotHeading);

        telemetry.update();
    }
}