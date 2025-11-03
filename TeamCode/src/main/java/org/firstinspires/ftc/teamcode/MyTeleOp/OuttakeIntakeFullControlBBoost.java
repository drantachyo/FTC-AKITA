package org.firstinspires.ftc.teamcode.MyTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "OuttakeIntakeFullControlBBoost_Servo", group = "TeleOp")
public class OuttakeIntakeFullControlBBoost extends OpMode {

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
    double rampPower = 0.0;
    final double MAX_RAMP_POWER = 0.8;
    final double POWER_STEP = 0.05;

    // --- Intake ---
    boolean intakeOn = false;
    boolean lastIntakeY = false;

    // --- B pulse control ---
    boolean bActive = false;
    long bStartTime = 0;

    // --- Servo control ---
    boolean servoOpen = false;
    boolean lastA = false;
    final double openPosition = 1.0;   // открыть
    final double closePosition = 0.65; // закрыть

    @Override
    public void init() {
        Outtake = hardwareMap.get(DcMotor.class, "Outtake");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        gateServo = hardwareMap.get(Servo.class, "GateServo");

        Outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        rampPower = startPower;

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));

        // серво по умолчанию закрыто
        gateServo.setPosition(closePosition);

        telemetry.addLine("Initialized Outtake + Intake + Servo");
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

        // --- B pulse intake ---
        if (currentB && !bActive) {
            bActive = true;
            bStartTime = System.currentTimeMillis();
            gateServo.setPosition(closePosition);  // закрыть в начале импульса
        }

        double intakePower = 0.0;
        double appliedOuttakePower = 0.0;

        // --- B pulse logic ---
        if (bActive) {
            long elapsed = System.currentTimeMillis() - bStartTime;
            if (elapsed < 150) {
                intakePower = 1.0;
                appliedOuttakePower = motorOn ? startPower : 0.0;
            } else if (elapsed < 800) {
                intakePower = 0.0;
                appliedOuttakePower = motorOn ? Math.min(startPower + 0.2, 1.0) : 0.0;
            } else if (elapsed < 1300) {
                intakePower = 1.0;
                appliedOuttakePower = motorOn ? startPower : 0.0;
            } else {
                bActive = false;
                gateServo.setPosition(openPosition); // открыть по окончанию импульса
            }
        } else {
            if (gamepad2.dpad_up) intakePower = -1.0;
            else if (gamepad2.dpad_down) intakePower = 1.0;
            else intakePower = intakeOn ? 1.0 : 0.0;

            if (motorOn) appliedOuttakePower = startPower;
            else appliedOuttakePower = 0.0;
        }

        Outtake.setPower(appliedOuttakePower);
        Intake.setPower(intakePower);

        // --- Save last states ---
        lastX = currentX;
        lastA = currentA;
        lastLB = currentLB;
        lastRB = currentRB;
        lastIntakeY = currentY;

        // --- Telemetry ---
        telemetry.addData("Outtake On", motorOn);
        telemetry.addData("Start Power", startPower);
        telemetry.addData("Applied Outtake Power", appliedOuttakePower);
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("B Pulse Active", bActive);
        telemetry.addData("Servo Open", servoOpen);
        telemetry.addData("Servo Position", gateServo.getPosition());
        telemetry.update();
    }
}
