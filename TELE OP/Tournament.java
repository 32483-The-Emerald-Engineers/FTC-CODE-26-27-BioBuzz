package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "---!Main Code!---", group = "TeleOp")
public class Tournament_Code extends LinearOpMode {

    // Drivetrain Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Intake Motor (Expansion Hub Port 0)
    private DcMotor intakeMotor;

    // Shooter Motor (Expansion Hub Port 2)
    private DcMotor shooter;

    // Vision (HuskyLens)
    private HuskyLens huskyLens;
    
    // Distance-based shooter power constants (using target width as proxy for distance/range)
    private final int MIN_TARGET_WIDTH = 30;   // Target far away (small width) -> power min
    private final int MAX_TARGET_WIDTH = 200;  // Target close (large width) -> power max
    private final double MIN_SHOOTER_POWER = 0.3;
    private final double MAX_SHOOTER_POWER = 1.0;

    // Intake toggle tracking state
    private boolean intakeOn = false;
    private boolean wasSquarePressed = false;

    @Override
    public void runOpMode() {
        // Initialize Drivetrain Motors from Hardware Map
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Initialize Intake Motor (Configured name in Robot Config for Expansion Hub Port 0)
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Initialize Shooter Motor (Expansion Hub Port 2)
        shooter = hardwareMap.get(DcMotor.class, "Shooter");

        // Reverse left side motors so positive power moves the robot forward
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Optional: Enable brake behavior for sharper stopping
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize HuskyLens on I2C
        huskyLens = hardwareMap.get(HuskyLens.class, "Camera");
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "Communication failed! Check connection/I2C port.");
        } else {
            telemetry.addData("HuskyLens", "Communicating properly");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData("Status", "Initialized & Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ==================== 1. DRIVETRAIN CONTROLS ====================
            // Standard Gamepad 1 drive controls with basic cubic scaling for fine control
            double forward = Math.pow(-gamepad1.left_stick_y, 3); // Reversed stick to match standard field directions
            double strafe = Math.pow(gamepad1.left_stick_x * 1.1, 3); // Counteract side-slip
            double rotate = Math.pow(gamepad1.right_stick_x, 3);

            // Calculate Mecanum power distribution
            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);
            double frontLeftPower = (forward + strafe + rotate) / denominator;
            double backLeftPower = (forward - strafe + rotate) / denominator;
            double frontRightPower = (forward - strafe - rotate) / denominator;
            double backRightPower = (forward + strafe - rotate) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // ==================== 2. INTAKE CONTROL (SQUARE / X BUTTON) ====================
            // Pressing Square (gamepad1.x) toggles intake state ON or OFF
            if (gamepad1.x && !wasSquarePressed) {
                intakeOn = !intakeOn;
            }
            wasSquarePressed = gamepad1.x; // Save current frame state for debouncing

            // Send power to intake motor based on state
            intakeMotor.setPower(intakeOn ? 1.0 : 0.0);

            // ==================== 3. HUSKYLENS APRILTAG DISTANCE & POSE SHOOTER CONTROL ====================
            HuskyLens.Block targetBlock = null;
            HuskyLens.Block[] blocks = huskyLens.blocks();

            // Detect ANY visible AprilTag block
            if (blocks != null && blocks.length > 0) {
                targetBlock = blocks[0]; // Lock onto the first detected AprilTag
            }

            double shooterPower = 0.0;
            double targetXOffset = 0.0;
            double targetYOffset = 0.0;

            if (targetBlock != null) {
                // Calculate estimated pose offsets (HuskyLens frame center is ~160px X, 120px Y)
                targetXOffset = targetBlock.x - 160; // Horizontal pixel offset from center
                targetYOffset = targetBlock.y - 120; // Vertical pixel offset from center

                // Calculate power based on target width (smaller width = further away = more power)
                double targetWidth = targetBlock.width;
                double powerRatio = (targetWidth - MIN_TARGET_WIDTH) / (double)(MAX_TARGET_WIDTH - MIN_TARGET_WIDTH);
                // Inverse relationship: smaller target width needs higher shooter power
                shooterPower = Range.clip(MAX_SHOOTER_POWER - powerRatio * (MAX_SHOOTER_POWER - MIN_SHOOTER_POWER), MIN_SHOOTER_POWER, MAX_SHOOTER_POWER);
            } else {
                // Shut down shooter when no target block is detected
                shooterPower = 0.0;
            }
            shooter.setPower(shooterPower);

            // Driver Telemetry Output
            telemetry.addData("Intake Status", intakeOn ? "RUNNING (100%)" : "STOPPED");
            telemetry.addData("Shooter Status", targetBlock != null ? String.format("ACTIVE (Power: %.2f)", shooterPower) : "OFF (No AprilTag)");
            if (targetBlock != null) {
                telemetry.addData("AprilTag ID", targetBlock.id);
                telemetry.addData("Pose X Offset", "%.1f px", targetXOffset);
                telemetry.addData("Pose Y Offset", "%.1f px", targetYOffset);
                telemetry.addData("Target Block Width", "%d px", targetBlock.width);
            }
            telemetry.addData("Drive Powers", "FL: %.2f | FR: %.2f | BL: %.2f | BR: %.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}
