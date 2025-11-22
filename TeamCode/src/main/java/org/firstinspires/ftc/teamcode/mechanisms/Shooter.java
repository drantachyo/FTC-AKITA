package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {

    private final DcMotorEx shooterMotor;
    private final Servo gateServo;

    private double targetRPM = 0.0;
    private double lastRPM = 0.0;

    // Для Boost режима
    private boolean forcedBoostActive = false;
    private long boostEndTime = 0;

    // PIDF коэффициенты – начальные (подгони под свой мотор)
    private final PIDFCoefficients pidf = new PIDFCoefficients(0.02, 0.0, 0.00005, 14.0);

    // Константы
    private static final double TICKS_PER_REV = 28; // проверь свой мотор
    private static final double MAX_RPM = 6000.0;

    // Servo позиции
    private static final double OPEN_POSITION = 0.65;
    private static final double CLOSE_POSITION = 1.0;

    // RPM recovery tolerance
    private static final double RPM_TOLERANCE = 50.0;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Outtake");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // применяем PIDF
        shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        gateServo = hardwareMap.get(Servo.class, "GateServo");
        gateServo.setPosition(CLOSE_POSITION);
    }

    // вызывать каждый цикл
    public void update() {
        long now = System.currentTimeMillis();

        // обработка форс-буста
        if (forcedBoostActive) {
            if (now < boostEndTime) {
                shooterMotor.setPower(1.0);
            } else {
                forcedBoostActive = false;
            }
        }

        // обычная поддержка скорости
        double targetTicksPerSecond = (targetRPM * TICKS_PER_REV) / 60.0;

        // Всегда ставим velocity, если targetRPM > 0
        if (targetRPM > 0) {
            shooterMotor.setVelocity(targetTicksPerSecond);
        } else {
            shooterMotor.setPower(0);
        }

        // обновим RPM для телеметрии
        double currentVelocity = shooterMotor.getVelocity(); // ticks/sec
        lastRPM = (currentVelocity * 60.0) / TICKS_PER_REV;
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = Math.max(0, Math.min(rpm, MAX_RPM));
    }

    public double getLastRPM() {
        return lastRPM;
    }

    public boolean isInRecovery() {
        return Math.abs(lastRPM - targetRPM) > RPM_TOLERANCE;
    }

    public void requestForcedBoost(long durationMs) {
        forcedBoostActive = true;
        boostEndTime = System.currentTimeMillis() + durationMs;
    }

    // управление сервой
    public void openGate() { gateServo.setPosition(OPEN_POSITION); }
    public void closeGate() { gateServo.setPosition(CLOSE_POSITION); }
    public double getGatePosition() { return gateServo.getPosition(); }

    // getter для прямого доступа к мотору (для диагностики)
    public DcMotorEx getMotor() { return shooterMotor; }

}
