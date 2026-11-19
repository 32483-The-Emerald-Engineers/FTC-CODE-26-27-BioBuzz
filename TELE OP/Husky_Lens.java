package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "HuskyLens Shooter Only", group = "TeleOp")
public class HuskyLensTest extends LinearOpMode {

    // Shooter Motor (Expansion Hub Port 2)
    private DcMotor shooter;

    // Vision (HuskyLens on I2C)
    private HuskyLens huskyLens;

    // Distance-based shooter power constants (using target width as a proxy for distance)
    private final int MIN_TARGET_WIDTH = 30;   // Target far away (small width) -> max power
    private final int MAX_TARGET_WIDTH = 200;  // Target close (large width) -> min power
    private final double MIN_SHOOTER_POWER = 0.3;
    private final double MAX_SHOOTER_POWER = 1.0;

    @Override
    public void runOpMode() {
        // Initialize Shooter Motor
        shooter = hardwareMap.get(DcMotor.class, "Shooter");

        // Initialize HuskyLens on I2C port named "Camera"
        huskyLens = hardwareMap.get(HuskyLens.class, "Camera");
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "Communication failed! Check I2C cable & config name.");
        } else {
            telemetry.addData("HuskyLens", "Connected!");
        }
        
        // Set HuskyLens to AprilTag / Tag Recognition mode
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData("Status", "HuskyLens Shooter Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
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
                // Calculate estimated pose offsets (HuskyLens screen center is 160px X, 120px Y)
                targetXOffset = targetBlock.x - 160;
                targetYOffset = targetBlock.y - 120;

                // Calculate shooter speed based on tag width (smaller width = further away = faster)
                double targetWidth = targetBlock.width;
                double powerRatio = (targetWidth - MIN_TARGET_WIDTH) / (double)(MAX_TARGET_WIDTH - MIN_TARGET_WIDTH);
                
                shooterPower = Range.clip(
                    MAX_SHOOTER_POWER - powerRatio * (MAX_SHOOTER_POWER - MIN_SHOOTER_POWER),
                    MIN_SHOOTER_POWER,
                    MAX_SHOOTER_POWER
                );
            } else {
                // Turn off shooter if no AprilTag is visible
                shooterPower = 0.0;
            }

            // Apply calculated power to the shooter motor
            shooter.setPower(shooterPower);

            // Telemetry for testing and tuning
            telemetry.addData("Shooter Status", targetBlock != null ? String.format("ACTIVE (Power: %.2f)", shooterPower) : "OFF (No AprilTag)");
            if (targetBlock != null) {
                telemetry.addData("AprilTag ID", targetBlock.id);
                telemetry.addData("Pose X Offset", "%.1f px", targetXOffset);
                telemetry.addData("Pose Y Offset", "%.1f px", targetYOffset);
                telemetry.addData("Tag Width", "%d px", targetBlock.width);
            }
            telemetry.update();
        }
    }
}
