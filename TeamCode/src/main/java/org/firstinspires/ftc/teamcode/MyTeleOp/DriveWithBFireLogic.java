package org.firstinspires.ftc.teamcode.MyTeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "DriveWithBFireLogic", group = "TeleOp")
public class DriveWithBFireLogic extends OpMode {

    // --- Шасси ---
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- Outtake / Intake / Servo ---
    DcMotor Outtake, Intake;
    Servo gateServo;
    IMU imu;

    // --- Outtake variables ---
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
    final double openPosition = 0.65;
    final double closePosition = 1.0;

    // --- Brake ---
    DcMotor.ZeroPowerBehavior prevFLZeroBehavior, prevFRZeroBehavior, prevBLZeroBehavior, prevBRZeroBehavior;
    boolean brakeModeActive = false;

    @Override
    public void init() {
        // --- Шасси ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        prevFLZeroBehavior = frontLeftDrive.getZeroPowerBehavior();
        prevFRZeroBehavior = frontRightDrive.getZeroPowerBehavior();
        prevBLZeroBehavior = backLeftDrive.getZeroPowerBehavior();
        prevBRZeroBehavior = backRightDrive.getZeroPowerBehavior();

        // --- Outtake / Intake / Servo ---
        Outtake = hardwareMap.get(DcMotor.class, "Outtake");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        gateServo = hardwareMap.get(Servo.class, "GateServo");

        Outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        gateServo.setPosition(closePosition);

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        telemetry.addLine("Initialized — ready to start!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Joysticks ---
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_trigger - gamepad1.left_trigger;

        if (gamepad1.options) imu.resetYaw();

        // D-pad приоритет
        if (gamepad1.dpad_up) { forward = 1.0; right = 0.0; }
        else if (gamepad1.dpad_down) { forward = -1.0; right = 0.0; }
        else if (gamepad1.dpad_left) { forward = 0.0; right = -1.0; }
        else if (gamepad1.dpad_right) { forward = 0.0; right = 1.0; }

        // --- Slow mode ---
        double speedMultiplier = gamepad1.right_bumper ? 0.2 : 1.0;

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

        // --- Gamepad 2 Controls ---
        boolean currentX = gamepad2.x;
        boolean currentA = gamepad2.a;
        boolean currentLB = gamepad2.left_bumper;
        boolean currentRB = gamepad2.right_bumper;
        boolean currentY = gamepad2.y;
        boolean currentB = gamepad2.b;

        // --- Toggle Outtake ---
        if (currentX && !lastX) motorOn = !motorOn;

        // --- Adjust startPower ---
        if (currentLB && !lastLB) startPower = Math.max(0.1, startPower - POWER_STEP);
        if (currentRB && !lastRB) startPower = Math.min(1.0, startPower + POWER_STEP);

        // --- Toggle Servo manually ---
        if (currentA && !lastA) {
            servoOpen = !servoOpen;
            gateServo.setPosition(servoOpen ? openPosition : closePosition);
        }

        // --- Toggle Intake ---
        if (currentY && !lastIntakeY) intakeOn = !intakeOn;

        // --- Start B pulse (fire sequence) ---
        if (currentB && !bActive) {
            bActive = true;
            bStartTime = System.currentTimeMillis();
        }

        double intakePower = 0.0;
        double outtakePower = motorOn ? startPower : 0.0;

        if (bActive) {
            long elapsed = System.currentTimeMillis() - bStartTime;
            double BOOST = Math.min(startPower + 0.16, 1.0);

            if (elapsed < 150) { gateServo.setPosition(openPosition); intakePower = 1.0; }
            else if (elapsed < 300) { intakePower = 0.0; }
            else if (elapsed < 500) { gateServo.setPosition(closePosition); }
            else if (elapsed < 700) { outtakePower = BOOST; }
            else if (elapsed < 900) { gateServo.setPosition(openPosition); }
            else if (elapsed < 1250) { intakePower = 1.0; }
            else if (elapsed < 1400) { intakePower = 0.0; }
            else { bActive = false; gateServo.setPosition(closePosition); outtakePower = startPower; }
        } else {
            if (gamepad2.dpad_up) intakePower = -1.0;
            else if (gamepad2.dpad_down) intakePower = 1.0;
            else intakePower = intakeOn ? 1.0 : 0.0;
        }

        // --- Apply powers ---
        Outtake.setPower(outtakePower);
        Intake.setPower(intakePower);

        // --- Update states ---
        lastX = currentX;
        lastA = currentA;
        lastLB = currentLB;
        lastRB = currentRB;
        lastIntakeY = currentY;

        // --- Telemetry ---
        telemetry.addData("Outtake On", motorOn);
        telemetry.addData("Start Power", startPower);
        telemetry.addData("Outtake Power", outtakePower);
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("B Pulse Active", bActive);
        telemetry.addData("Servo Pos", gateServo.getPosition());
        telemetry.addData("Slow Mode (RB1)", speedMultiplier < 1.0);
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
