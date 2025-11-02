package org.firstinspires.ftc.teamcode.MyTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "OuttakeRampWithToggle", group = "TeleOp")
public class OuttakeRampWithToggle extends OpMode {

    DcMotor Outtake;

    // Старт/стоп мотора
    boolean motorOn = false;
    boolean lastX = false;

    // Разгон на кнопку A
    boolean rampActive = false;
    boolean lastA = false;
    boolean lastLB = false;
    boolean lastRB = false;

    double startPower = 0.55; // стартовая мощность (LB/RB регулирует)
    double rampPower = 0.0;   // текущая мощность при плавном разгоне
    final double MAX_RAMP_POWER = 0.8; // максимум разгона
    final double POWER_STEP = 0.05;    // LB/RB шаг настройки стартовой

    @Override
    public void init() {
        Outtake = hardwareMap.get(DcMotor.class, "Outtake");
        Outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        rampPower = startPower;
    }

    @Override
    public void loop() {
        boolean currentX = gamepad2.x;
        boolean currentA = gamepad2.a;
        boolean currentLB = gamepad2.left_bumper;
        boolean currentRB = gamepad2.right_bumper;

        // --- Toggle мотор (X) ---
        if (currentX && !lastX) {
            motorOn = !motorOn;
        }

        // --- Настройка стартовой мощности (LB/RB) ---
        if (currentLB && !lastLB) {
            startPower = Math.max(0.1, startPower - POWER_STEP);
        }
        if (currentRB && !lastRB) {
            startPower = Math.min(1.0, startPower + POWER_STEP);
        }

        // --- Разгон по кнопке A ---
        if (currentA && !lastA) {
            rampActive = true;
            rampPower = startPower; // начинаем с выбранной стартовой мощности
        }
        if (!currentA) {
            rampActive = false;
        }

        // --- Вычисление мощности ---
        double appliedPower = 0.0;
        if (motorOn) {
            if (rampActive && rampPower < MAX_RAMP_POWER) {
                rampPower += 0.003; // плавный прирост
            }
            appliedPower = rampActive ? rampPower : startPower;
            Outtake.setPower(appliedPower);
        } else {
            appliedPower = 0.0;
            Outtake.setPower(0.0);
        }

        lastX = currentX;
        lastA = currentA;
        lastLB = currentLB;
        lastRB = currentRB;

        telemetry.addData("Motor On", motorOn);
        telemetry.addData("Ramp Active", rampActive);
        telemetry.addData("Start Power", startPower);
        telemetry.addData("Ramp Power", rampPower);
        telemetry.addData("Applied Power", appliedPower); // выводим реально подаваемую мощность
        telemetry.update();
    }
}
