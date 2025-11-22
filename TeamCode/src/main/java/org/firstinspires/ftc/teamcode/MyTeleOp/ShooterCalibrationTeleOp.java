package org.firstinspires.ftc.teamcode.MyTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@TeleOp(name="Shooter Calibration", group="Test")
public class ShooterCalibrationTeleOp extends OpMode {

    Shooter shooter;
    double targetRPM = 3000; // можно менять с джойстика
    double lastRPM = 0;

    long recoveryStartTime = 0;
    long recoveryTime = 0;
    double deltaRPM = 0;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        telemetry.addLine("Shooter Calibration Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Управление targetRPM джойстиком ---
        if (gamepad1.dpad_up) targetRPM += 50;
        if (gamepad1.dpad_down) targetRPM -= 50;
        targetRPM = Math.max(0, targetRPM);

        shooter.setTargetRPM(targetRPM);
        shooter.update();

        lastRPM = shooter.getLastRPM();

        // --- Проверка восстановления RPM после просадки ---
        double rpmDifference = targetRPM - lastRPM;

        if (rpmDifference > 50) { // RPM упала сильно — старт восстановления
            if (recoveryStartTime == 0) recoveryStartTime = System.currentTimeMillis();
        } else {
            if (recoveryStartTime != 0) {
                recoveryTime = System.currentTimeMillis() - recoveryStartTime;
                deltaRPM = targetRPM - lastRPM;
                telemetry.addData("RecoveryTime (ms)", recoveryTime);
                telemetry.addData("DeltaRPM", deltaRPM);
                recoveryStartTime = 0; // сбросим для следующего выстрела
            }
        }

        // --- Telemetry ---
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Measured RPM", lastRPM);
        telemetry.addData("In Recovery?", rpmDifference > 50);
        telemetry.addData("RecoveryTime (ms)", recoveryTime);
        telemetry.addData("DeltaRPM", deltaRPM);
        telemetry.update();
    }
}
