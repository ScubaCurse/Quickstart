package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.commands.Shoot;

@TeleOp

public class TwoPlayer extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motorTest;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest;
    private int shooterVelocity;

    /*
    What we need to do is get the robot position from Pedro.
    double robotX = follower.getPose().getX();
    double robotY = follower.getPose().getY();

    Get the hive coordinates.
    double hiveX = 58;
    double hiveY = 50;

    Calculate the distance
    double distance = Math.hypot(
        hiveX - robotX,
        hiveY - robotY
    );
    Calculate the target RPM
    double targetRPM = ShootCalc.getTargetRPM(distance);

    Update shooter velocity
    Shoot.shoot(targetRPM);
     */
    @Override
    public void runOpMode() {

        shooterVelocity = 3000; // rpm

        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}