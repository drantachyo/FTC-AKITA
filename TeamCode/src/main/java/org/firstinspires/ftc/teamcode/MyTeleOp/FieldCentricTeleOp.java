package org.firstinspires.ftc.teamcode.MyTeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentric", group = "TeleOp")
public class FieldCentricTeleOp extends OpMode {

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor Outtake;
    DcMotor Intake;

    IMU imu;

    // Outtake
    boolean outtakeOn = false;
    boolean idleMode = false;
    boolean lastA = false;
    boolean lastY = false;
    boolean lastLB = false;
    boolean lastRB = false;

    // Intake
    boolean intakeOn = false;
    boolean lastX = false;

    double outtakePower = 0.3;
    final double MIN_POWER = 0.1;
    final double MAX_POWER = 1.0;
    final double POWER_STEP = 0.05;

    // Для сохранения предыдущего поведения моторов (чтобы восстановить после отпускания B)
    DcMotor.ZeroPowerBehavior prevFLZeroBehavior;
    DcMotor.ZeroPowerBehavior prevFRZeroBehavior;
    DcMotor.ZeroPowerBehavior prevBLZeroBehavior;
    DcMotor.ZeroPowerBehavior prevBRZeroBehavior;

    boolean brakeModeActive = false; // флаг, что сейчас удерживается B и активен брейк

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backLeftDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");
        Outtake = hardwareMap.get(DcMotor.class, "Outtake");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        // Настройка направлений
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        Outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Сохраняем текущее ZeroPowerBehavior
        prevFLZeroBehavior = frontLeftDrive.getZeroPowerBehavior();
        prevFRZeroBehavior = frontRightDrive.getZeroPowerBehavior();
        prevBLZeroBehavior = backLeftDrive.getZeroPowerBehavior();
        prevBRZeroBehavior = backRightDrive.getZeroPowerBehavior();

        // Инициализация IMU
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
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (gamepad1.options) imu.resetYaw();

        // ----- OUTTAKE: A toggle, Y idle mode, LB/RB adjust (gamepad2) -----
        boolean currentA = gamepad2.a;
        boolean currentY = gamepad2.y;
        boolean currentLB = gamepad2.left_bumper;
        boolean currentRB = gamepad2.right_bumper;
        boolean currentX = gamepad2.x;

        // A — тумблер вкл/выкл Outtake, выход из idle режима
        if (currentA && !lastA) {
            outtakeOn = !outtakeOn;
            idleMode = false;
        }

        // Y — холостой режим 0.1
        if (currentY && !lastY) {
            outtakeOn = true;
            idleMode = true;
        }

        // RB (gamepad2) +0.1, LB (gamepad2) -0.1
        if (currentRB && !lastRB) {
            outtakePower = Math.min(MAX_POWER, outtakePower + POWER_STEP);
            idleMode = false;
        }
        if (currentLB && !lastLB) {
            outtakePower = Math.max(MIN_POWER, outtakePower - POWER_STEP);
            idleMode = false;
        }

        // Intake toggle (X)
        if (currentX && !lastX) {
            intakeOn = !intakeOn;
        }

        // Применяем мощность на Outtake
        if (!outtakeOn) {
            Outtake.setPower(0.0);
        } else if (idleMode) {
            Outtake.setPower(0.1);
        } else {
            Outtake.setPower(outtakePower);
        }

        // Применяем мощность на Intake
        Intake.setPower(intakeOn ? 1.0 : 0.0);

        lastA = currentA;
        lastY = currentY;
        lastLB = currentLB;
        lastRB = currentRB;
        lastX = currentX;

        // ----- Движение: slow mode RB (gamepad1), brake B (gamepad1) -----
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_trigger - gamepad1.left_trigger;

        // D-pad приоритет
        if (gamepad1.dpad_up) {
            forward = 1.0;
            right = 0.0;
        } else if (gamepad1.dpad_down) {
            forward = -1.0;
            right = 0.0;
        } else if (gamepad1.dpad_left) {
            forward = 0.0;
            right = -1.0;
        } else if (gamepad1.dpad_right) {
            forward = 0.0;
            right = 1.0;
        }

        // Slow mode — правая кнопка-бампер первого геймпада (умножение скорости)
        boolean slowMode = gamepad1.right_bumper;
        double speedMultiplier = slowMode ? 0.2 : 1.0;

        // Brake
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
            driveFieldRelative(forward * speedMultiplier, right * speedMultiplier, rotate * 0.8 * speedMultiplier);
        }

        telemetry.addData("Heading (Yaw)", currentYaw);
        telemetry.addData("Outtake On", outtakeOn);
        telemetry.addData("Idle Mode", idleMode);
        telemetry.addData("Target Power", outtakePower);
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Slow Mode (RB1)", slowMode);
        telemetry.addData("Brake Active (B1)", brakeModeActive);
        telemetry.update();
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backRightPower), Math.abs(backLeftPower)))));

        frontLeftDrive.setPower(frontLeftPower / maxPower);
        frontRightDrive.setPower(frontRightPower / maxPower);
        backLeftDrive.setPower(backLeftPower / maxPower);
        backRightDrive.setPower(backRightPower / maxPower);
    }
}
