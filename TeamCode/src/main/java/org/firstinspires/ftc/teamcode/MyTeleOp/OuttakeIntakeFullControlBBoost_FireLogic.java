package org.firstinspires.ftc.teamcode.MyTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "OuttakeIntakeFullControlBBoost_FireLogic", group = "TeleOp")
public class OuttakeIntakeFullControlBBoost_FireLogic extends OpMode {

    DcMotor Outtake;
    DcMotor Intake;
    Servo gateServo;
    IMU imu;

    // --- Outtake ---
    boolean motorOn = false;
    boolean lastX = false;
    boolean lastLB = false;
    boolean lastRB = false;

    double startPower = 0.55;
    final double POWER_STEP = 0.05;

    // --- Intake ---
    boolean intakeOn = false;
    boolean lastIntakeY = false;

    // --- B pulse ---
    boolean bActive = false;
    long bStartTime = 0;

    // --- Servo ---
    boolean servoOpen = false;
    boolean lastA = false;
    final double openPosition = 0.65;   // открыть
    final double closePosition = 1.0;   // закрыть

    @Override
    public void init() {
        Outtake = hardwareMap.get(DcMotor.class, "Outtake");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        gateServo = hardwareMap.get(Servo.class, "GateServo");

        Outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        gateServo.setPosition(closePosition);

        telemetry.addLine("Initialized Outtake + Intake + Gate Servo");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean currentX = gamepad2.x;
        boolean currentA = gamepad2.a;
        boolean currentLB = gamepad2.left_bumper;
        boolean currentRB = gamepad2.right_bumper;
        boolean currentY = gamepad2.y;
        boolean currentB = gamepad2.b;

        // --- Toggle Outtake (X) ---
        if (currentX && !lastX) motorOn = !motorOn;

        // --- Adjust startPower (LB/RB) ---
        if (currentLB && !lastLB) startPower = Math.max(0.1, startPower - POWER_STEP);
        if (currentRB && !lastRB) startPower = Math.min(1.0, startPower + POWER_STEP);

        // --- Toggle Servo manually (A) ---
        if (currentA && !lastA) {
            servoOpen = !servoOpen;
            gateServo.setPosition(servoOpen ? openPosition : closePosition);
        }

        // --- Toggle Intake (Y) ---
        if (currentY && !lastIntakeY) intakeOn = !intakeOn;

        // --- Start B pulse (fire sequence) ---
        if (currentB && !bActive) {
            bActive = true;
            bStartTime = System.currentTimeMillis();
        }

        double intakePower = 0.0;
        double outtakePower = motorOn ? startPower : 0.0;

        // --- B-pulse timed logic (same as fireBpulse) ---
        if (bActive) {
            long elapsed = System.currentTimeMillis() - bStartTime;
            double BOOST = Math.min(startPower + 0.2, 1.0);

            if (elapsed < 150) {
                // (1) открыть гейт, (2) подать коротко
                gateServo.setPosition(openPosition);
                intakePower = 1.0;
            } else if (elapsed < 300) {
                // закончить короткую подачу
                intakePower = 0.0;
            } else if (elapsed < 500) {
                // (3) закрыть гейт
                gateServo.setPosition(closePosition);
            } else if (elapsed < 700) {
                // (4) поднять мощность
                outtakePower = BOOST;
            } else if (elapsed < 900) {
                // (5) открыть гейт — выстрел
                gateServo.setPosition(openPosition);
            } else if (elapsed < 1250) {
                // (6) подать второй мяч
                intakePower = 1.0;
            } else if (elapsed < 1400) {
                intakePower = 0.0;
            } else {
                // (7) вернуть всё
                bActive = false;
                gateServo.setPosition(closePosition);
                outtakePower = startPower;
            }
        } else {
            // --- Обычное управление ---
            if (gamepad2.dpad_up) intakePower = -1.0;
            else if (gamepad2.dpad_down) intakePower = 1.0;
            else intakePower = intakeOn ? 1.0 : 0.0;

            if (!motorOn) outtakePower = 0.0;
        }

        // --- Применение мощности ---
        Outtake.setPower(outtakePower);
        Intake.setPower(intakePower);

        // --- Обновление состояний ---
        lastX = currentX;
        lastA = currentA;
        lastLB = currentLB;
        lastRB = currentRB;
        lastIntakeY = currentY;

        // --- Телеметрия ---
        telemetry.addData("Outtake On", motorOn);
        telemetry.addData("Start Power", startPower);
        telemetry.addData("Outtake Power", outtakePower);
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("B Pulse Active", bActive);
        telemetry.addData("Servo Pos", gateServo.getPosition());
        telemetry.update();
    }
}
