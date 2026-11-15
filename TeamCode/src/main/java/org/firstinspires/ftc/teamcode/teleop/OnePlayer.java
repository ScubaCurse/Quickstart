package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "OnePlayer")
public class OnePlayer extends LinearOpMode {

    private DcMotor FL;
    private DcMotor RL;
    private DcMotor FR;
    private DcMotor RR;
    private DcMotor intake;
    private DcMotorEx shooterRight;
    private DcMotorEx shooterLeft;

    double speedMode;
    double frontLeftPower;
    double backLeftPower;
    ElapsedTime runtime;
    double frontRightPower;
    double backRightPower;
    @Override
    public void runOpMode() {
        int shooterVelocity;
        double avgShooterSpeed;

        FL = hardwareMap.get(DcMotor.class, "FL");
        RL = hardwareMap.get(DcMotor.class, "RL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        RR = hardwareMap.get(DcMotor.class, "RR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();

        speedMode = 3;
        shooterVelocity = 1650;
        runtime = new ElapsedTime();
        FL.setDirection(DcMotor.Direction.FORWARD);
        RL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses START)
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ((DcMotorEx) shooterLeft).setVelocity(0);
        ((DcMotorEx) shooterRight).setVelocity(0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            mecanum();
            // SHOOTER CONTROL

            if (gamepad1.right_bumper) {

                // Main shooting velocity
                shooterLeft.setVelocity(shooterVelocity);
                shooterRight.setVelocity(shooterVelocity);

                if (gamepad1.right_trigger > 0) {
                    intake.setPower(0.35);
                } else {
                    intake.setPower(0);
                }

            } else if (gamepad1.left_trigger > 0) {

                // Reverse shooter
                shooterLeft.setVelocity(-1000);
                shooterRight.setVelocity(-1000);
                intake.setPower(1);

            } else if (gamepad1.left_bumper) {

                // Reverse intake
                shooterLeft.setVelocity(0);
                shooterRight.setVelocity(0);
                intake.setPower(-0.25);
            } else {

                shooterLeft.setVelocity(0);
                shooterRight.setVelocity(0);
                intake.setPower(0);
            }
            if (gamepad1.dpad_up) {
                shooterVelocity = shooterVelocity + 25;
            } else if (gamepad1.dpad_down) {
                shooterVelocity = shooterVelocity -25;
            }
            // Average out the speed of the left and right shooters.
            avgShooterSpeed = (((DcMotorEx) shooterLeft).getVelocity() + ((DcMotorEx) shooterRight).getVelocity()) / 2;
            // Print the set shooter velocity
            telemetry.addData("ShooterVelocity", shooterVelocity);
            // Print out the percentage that the shooters have reached towards the set velocity.
            telemetry.addData("ShooterLV", ((DcMotorEx) shooterLeft).getVelocity());
            telemetry.addData("ShooterRV", ((DcMotorEx) shooterRight).getVelocity());

            telemetry.update();
            if (gamepad1.x) {
                speedMode = 3;
            }
            if (gamepad1.y) {
                speedMode = 0.001;
            }

            // telemetry.addData("SpeedMode:", speedMode);
        }
    }

    /**
     * Describe this function...
     */
    private void mecanum() {
        double axial;
        float lateral;
        double yaw;
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        // Note: pushing stick forward gives negative value
        axial = -(gamepad1.left_stick_y / speedMode);
        lateral = gamepad1.right_stick_x / 2;
        yaw = gamepad1.left_stick_x / speedMode;
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        frontLeftPower = axial + lateral + yaw;
        frontRightPower = (axial - lateral) - yaw;
        backLeftPower = (axial + lateral) - yaw;
        backRightPower = (axial - lateral) + yaw;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1) {
            frontLeftPower = frontLeftPower / max;
            frontRightPower = frontRightPower / max;
            backLeftPower = backLeftPower / max;
            backRightPower = backRightPower / max;
        }
        // Send calculated power to wheels.
        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        RL.setPower(backLeftPower);
        RR.setPower(backRightPower);
        /*
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Front left/Right", JavaUtil.formatNumber(frontLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(frontRightPower, 4, 2));
        telemetry.addData("Back  left/Right", JavaUtil.formatNumber(backLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(backRightPower, 4, 2));
        telemetry.update();
         */
    }

    /**
     * This function is used to test your motor directions.
     *
     * Each button should make the corresponding motor run FORWARD.
     *
     *   1) First get all the motors to take to correct positions on the robot
     *      by adjusting your Robot Configuration if necessary.
     *
     *   2) Then make sure they run in the correct direction by modifying the
     *      the setDirection() calls above.
     */
    private void testMotorDirections() {
        frontLeftPower = gamepad1.x ? 1 : 0;
        backLeftPower = gamepad1.a ? 1 : 0;
        frontRightPower = gamepad1.y ? 1 : 0;
        backRightPower = gamepad1.b ? 1 : 0;
    }
}