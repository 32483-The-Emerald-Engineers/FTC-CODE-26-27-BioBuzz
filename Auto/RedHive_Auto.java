package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AIOptimizedAuto", group = "Autonomous")
public class AIOptimizedAuto extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    private double currentX = 0.0;
    private double currentY = 0.0;
    private double currentAngle = 0.0;

    private static final double TICKS_PER_INCH = 45.28;
    private static final double TICKS_PER_DEGREE = 8.35;

    @Override
    public void runOpMode() {
        leftFront  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        if (opModeIsActive()) {
            currentX = 144.5; currentY = 58.2; currentAngle = 0;
            // --- BEGIN AI SMOOTHED PATH TRAJECTORY ---
            executeMove(121.0, 57.7, 90.0, 0.50);
            executeMove(96.5, 13.3, -180.0, 0.50);
            executeMove(144.2, 57.2, 90.0, 0.50);
            executeMove(120.7, 95.1, -90.0, 0.50);
            executeMove(140.2, 95.1, -90.0, 0.50);
            executeMove(144.2, 57.5, 90.0, 0.50);
            executeMove(48.0, 9.7, 100.0, 0.50);
            // --- END AI SMOOTHED PATH TRAJECTORY ---
        }
    }

    public void executeMove(double targetX, double targetY, double targetAngle, double power) {
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double deltaAngle = targetAngle - currentAngle;

        while (deltaAngle < -180) deltaAngle += 360;
        while (deltaAngle > 180)  deltaAngle -= 360;

        int moveTicksX = (int)(deltaX * TICKS_PER_INCH);
        int moveTicksY = (int)(deltaY * TICKS_PER_INCH);
        int turnTicks  = (int)(deltaAngle * TICKS_PER_DEGREE);

        int targetLF = leftFront.getCurrentPosition()  + (moveTicksY + moveTicksX + turnTicks);
        int targetRF = rightFront.getCurrentPosition() + (moveTicksY - moveTicksX - turnTicks);
        int targetLB = leftBack.getCurrentPosition()   + (moveTicksY - moveTicksX + turnTicks);
        int targetRB = rightBack.getCurrentPosition()  + (moveTicksY + moveTicksX - turnTicks);

        leftFront.setTargetPosition(targetLF);
        rightFront.setTargetPosition(targetRF);
        leftBack.setTargetPosition(targetLB);
        rightBack.setTargetPosition(targetRB);

        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power); rightFront.setPower(power);
        leftBack.setPower(power);  rightBack.setPower(power);

        while (opModeIsActive() && (leftFront.isBusy() && rightFront.isBusy())) {
            idle();
        }

        currentX = targetX; currentY = targetY; currentAngle = targetAngle;
    }

    private void setEncoderMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);  rightFront.setMode(mode);
        leftBack.setMode(mode);   rightBack.setMode(mode);
    }
}
