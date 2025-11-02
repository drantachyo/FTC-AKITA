package org.firstinspires.ftc.teamcode.MyTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "AKITAA~``", group = "TeleOp")
public class AKITA extends OpMode {

    // --- Шасси ---
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- Механизмы ---
    DcMotor Outtake, Intake;
    IMU imu;

    // Outtake
    boolean motorOn = false, rampActive = false;
    double startPower = 0.55, rampPower = 0.0;
    final double MAX_RAMP_POWER = 0.8, POWER_STEP = 0.05;

    // Intake
    boolean intakeOn = false;

    // Шасси
    boolean brakeActive = false;

    @Override
    public void init() {
        // Hardware mapping здесь
    }

    @Override
    public void loop() {
        // --- Field-centric ---
        // if (gamepad1.options) imu.resetYaw();

        // --- Outtake ---
        // Toggle, ramp, LB/RB регулировка мощности

        // --- Intake ---
        // Toggle

        // --- Drive ---
        // Управление шасси, slow mode, brake, field-centric

        // --- Telemetry ---
    }

    // --- Функции ---
    public void driveFieldRelative(double forward, double right, double rotate) { /* ... */ }
    public void drive(double forward, double right, double rotate) { /* ... */ }
    public void applyOuttakePower() { /* ... */ }
    public void toggleIntake() { /* ... */ }
}
