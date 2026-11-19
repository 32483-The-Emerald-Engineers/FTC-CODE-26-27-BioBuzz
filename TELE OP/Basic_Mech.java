package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Basic Mecanum Drive", group = "TeleOp")
public class MecanumDrive extends LinearOpMode {

    // Drivetrain Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Speed Multiplier (1.0 = 100% Max Speed)
    private final double DRIVE_SPEED_MULTIPLIER = 1.0;

    @Override
    public void runOpMode() {
        // Initialize Drivetrain Motors from Hardware Map
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse left side motors so positive power moves the robot forward
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Enable brake behavior for sharper stopping
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Mecanum Drive Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read Gamepad 1 joysticks with cubic scaling for smoother low-speed control
            double forward = Math.pow(-gamepad1.left_stick_y, 3); // Reversed so up on stick is positive
            double strafe = Math.pow(gamepad1.left_stick_x * 1.1, 3); // 1.1x multiplier counteracts side-slip
            double rotate = Math.pow(gamepad1.right_stick_x, 3);

            // Calculate Mecanum power distribution & normalize so no motor exceeds 1.0
            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);
            double frontLeftPower = ((forward + strafe + rotate) / denominator) * DRIVE_SPEED_MULTIPLIER;
            double backLeftPower = ((forward - strafe + rotate) / denominator) * DRIVE_SPEED_MULTIPLIER;
            double frontRightPower = ((forward - strafe - rotate) / denominator) * DRIVE_SPEED_MULTIPLIER;
            double backRightPower = ((forward + strafe - rotate) / denominator) * DRIVE_SPEED_MULTIPLIER;

            // Apply power to drive motors
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Driver Telemetry
            telemetry.addData("Drive Powers", "FL: %.2f | FR: %.2f | BL: %.2f | BR: %.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}
