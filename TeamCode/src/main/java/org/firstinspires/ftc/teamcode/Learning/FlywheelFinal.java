package org.firstinspires.ftc.teamcode.Learning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Flywheel_Final_Version_LB_RB", group = "TeleOp")
public class FlywheelFinal extends OpMode {

    DcMotorEx fly;
    DcMotor intake;

    // PIDF
    double kP = 25.0;
    double kD = 2.0;
    double kF = 14.0;

    // RPM
    double targetRPM = 3000;
    double targetTPS;

    // Boost
    double boostPower = 0.15;  // +15%
    long boostTime = 120;
    long boostStart = 0;

    double dropThreshold = 250;

    // --- Для дебаунса кнопок ---
    boolean lastLB = false;
    boolean lastRB = false;
    final double RPM_STEP = 100;

    @Override
    public void init() {

        fly = hardwareMap.get(DcMotorEx.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // calculate TPS
        targetTPS = targetRPM / 60.0 * 28.0;

        // set PIDF once
        fly.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, 0, kD, kF)
        );

        telemetry.addData("Status", "Ready");
    }

    @Override
    public void loop() {

        // ===========================
        //  RPM CONTROL BY LB/RB
        // ===========================
        boolean currentLB = gamepad1.left_bumper;
        boolean currentRB = gamepad1.right_bumper;

        if (currentLB && !lastLB) targetRPM = Math.max(500, targetRPM - RPM_STEP);
        if (currentRB && !lastRB) targetRPM = targetRPM + RPM_STEP;

        lastLB = currentLB;
        lastRB = currentRB;

        targetTPS = targetRPM / 60.0 * 28.0;

        // ===========================
        //  BOOST LOGIC
        // ===========================
        double currentRPM = fly.getVelocity() / 28.0 * 60.0;

        if (targetRPM - currentRPM > dropThreshold) {
            boostStart = System.currentTimeMillis();
        }

        boolean boosting =
                (System.currentTimeMillis() - boostStart) < boostTime;

        double velocityToSet = targetTPS;

        if (boosting) {
            velocityToSet *= (1.0 + boostPower);
        }

        fly.setVelocity(velocityToSet);

        // ===========================
        //  INTAKE ALWAYS ON
        // ===========================
        intake.setPower(1.0);

        // ===========================
        //  TELEMETRY
        // ===========================
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Boosting", boosting);
        telemetry.addData("Intake", "ON");
        telemetry.update();
    }

}
