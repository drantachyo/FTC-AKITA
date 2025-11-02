package org.firstinspires.ftc.teamcode.MyTeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TEST KOTAK AKHMEDI", group = "TeleOp")
public class TESTKOTAK extends OpMode {

    // Шасси
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    // Ауттейк
    DcMotor Outtake;
    // IMU для field-centric
    IMU imu;

    // --- Outtake variables ---
    boolean motorOn = false;
    boolean rampActive = false;
    boolean lastX = false;
    boolean lastA = false;
    double startPower = 0.55;        // регулируется LB/RB
    double rampPower = 0.0;
    final double MAX_RAMP_POWER = 0.8;
    final double POWER_STEP = 0.05;
    boolean lastLB = false;
    boolean lastRB = false;


    // --- Шасси ---
    DcMotor.ZeroPowerBehavior prevFLZeroBehavior, prevFRZeroBehavior, prevBLZeroBehavior, prevBRZeroBehavior;
    boolean brakeModeActive = false;

    @Override
    public void init() {
        // --- Шасси ---


        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backLeftDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        prevFLZeroBehavior = frontLeftDrive.getZeroPowerBehavior();
        prevFRZeroBehavior = frontRightDrive.getZeroPowerBehavior();
        prevBLZeroBehavior = backLeftDrive.getZeroPowerBehavior();
        prevBRZeroBehavior = backRightDrive.getZeroPowerBehavior();

        // --- Outtake ---
        Outtake = hardwareMap.get(DcMotor.class, "Outtake");
        Outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        rampPower = startPower;

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));

        telemetry.addLine("Initialized — ready to start!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Joysticks ---
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_trigger - gamepad1.left_trigger;
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // D-pad приоритет
        if (gamepad1.dpad_up) { forward = 1.0; right = 0.0; }
        else if (gamepad1.dpad_down) { forward = -1.0; right = 0.0; }
        else if (gamepad1.dpad_left) { forward = 0.0; right = -1.0; }
        else if (gamepad1.dpad_right) { forward = 0.0; right = 1.0; }

        // --- Slow mode ---
        boolean slowMode = gamepad1.right_bumper;
        double speedMultiplier = slowMode ? 0.2 : 1.0;

        // --- Brake ---
        if (gamepad1.b) {
            if (!brakeModeActive) {
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                brakeModeActive = true;
            }
            frontLeftDrive.setPower(0.0);
            frontRightDrive.setPower(0.0);
            backLeftDrive.setPower(0.0);
            backRightDrive.setPower(0.0);
        } else {
            if (brakeModeActive) {
                frontLeftDrive.setZeroPowerBehavior(prevFLZeroBehavior);
                frontRightDrive.setZeroPowerBehavior(prevFRZeroBehavior);
                backLeftDrive.setZeroPowerBehavior(prevBLZeroBehavior);
                backRightDrive.setZeroPowerBehavior(prevBRZeroBehavior);
                brakeModeActive = false;
            }
            driveFieldRelative(forward * speedMultiplier, right * speedMultiplier, rotate * speedMultiplier * 0.8);
        }

        // --- Outtake control ---
        boolean currentX = gamepad2.x;
        boolean currentA = gamepad2.a;
        boolean currentLB = gamepad2.left_bumper;
        boolean currentRB = gamepad2.right_bumper;

        if (currentX && !lastX) motorOn = !motorOn;
        if (currentA && !lastA) { rampActive = true; rampPower = startPower; }
        if (!currentA) rampActive = false;
        if (currentLB && !lastLB) startPower = Math.max(0.1, startPower - POWER_STEP);
        if (currentRB && !lastRB) startPower = Math.min(1.0, startPower + POWER_STEP);

        double appliedPower = 0.0;
        if (motorOn) {
            if (rampActive && rampPower < MAX_RAMP_POWER) rampPower += 0.003;
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

        // --- Telemetry ---
        telemetry.addData("Motor On", motorOn);
        telemetry.addData("Ramp Active", rampActive);
        telemetry.addData("Start Power", startPower);
        telemetry.addData("Ramp Power", rampPower);
        telemetry.addData("Applied Power", appliedPower);
        telemetry.addData("Slow Mode (RB1)", slowMode);
        telemetry.addData("Brake Active (B1)", brakeModeActive);
        telemetry.update();
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        drive(newForward, newRight, rotate);
    }

    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double br = forward + right - rotate;
        double bl = forward - right + rotate;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(br), Math.abs(bl)))));
        frontLeftDrive.setPower(fl / max);
        frontRightDrive.setPower(fr / max);
        backLeftDrive.setPower(bl / max);
        backRightDrive.setPower(br / max);
    }
}
