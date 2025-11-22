package org.firstinspires.ftc.teamcode.Learning;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Drive + PID Shoot Fixed", group = "TeleOp")
public class DriveWithPIDShoot extends OpMode {
    // --- Шасси ---
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- Outtake / Intake / Servo ---
    DcMotorEx Outtake;
    DcMotor Intake;
    Servo gateServo;
    IMU imu;

    // --- PIDF настройки ---
    double kP = 20.0, kI = 0, kD = 2.0, kF = 14.0;
    double targetRPM = 3000;
    double targetTPS;
    double boostPower = 0.15;  // +15%
    long boostTime = 120;
    long boostStart = 0;
    double dropThreshold = 250;

    // --- Intake / Servo flags ---
    boolean intakeOn = false;
    boolean servoOpen = false;
    boolean lastX=false, lastA=false, lastY=false, lastB=false;
    boolean lastLB = false, lastRB = false;
    boolean shooterOn = false;

    final double openPosition = 0.65;
    final double closePosition = 1.0;

    // --- Brake ---
    DcMotor.ZeroPowerBehavior prevFLZeroBehavior, prevFRZeroBehavior, prevBLZeroBehavior, prevBRZeroBehavior;
    boolean brakeModeActive = false;

    // --- B button sequence ---
    enum BState { OPEN_SERVO, INTAKE, DONE }
    BState bState = BState.DONE;
    long bSequenceStart = 0;

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
        Outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        gateServo = hardwareMap.get(Servo.class, "GateServo");

        Outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        gateServo.setPosition(closePosition);

        Outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetTPS = targetRPM / 60.0 * 28.0;

        Outtake.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        telemetry.addLine("Initialized — ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Joysticks ---
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_trigger - gamepad1.left_trigger;

        if(gamepad1.options) imu.resetYaw();

        if(gamepad1.dpad_up){ forward = 1.0; right = 0.0; }
        else if(gamepad1.dpad_down){ forward = -1.0; right = 0.0; }
        else if(gamepad1.dpad_left){ forward = 0.0; right = -1.0; }
        else if(gamepad1.dpad_right){ forward = 0.0; right = 1.0; }

        double speedMultiplier = gamepad1.right_bumper ? 0.2 : 1.0;

        // --- Brake ---
        if(gamepad1.b){
            if(!brakeModeActive){
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                brakeModeActive = true;
            }
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
        } else {
            if(brakeModeActive){
                frontLeftDrive.setZeroPowerBehavior(prevFLZeroBehavior);
                frontRightDrive.setZeroPowerBehavior(prevFRZeroBehavior);
                backLeftDrive.setZeroPowerBehavior(prevBLZeroBehavior);
                backRightDrive.setZeroPowerBehavior(prevBRZeroBehavior);
                brakeModeActive = false;
            }
            driveFieldRelative(forward*speedMultiplier, right*speedMultiplier, rotate*speedMultiplier*0.8);
        }

        // --- Gamepad 2 Controls ---
        boolean currentX = gamepad2.x;
        boolean currentA = gamepad2.a;
        boolean currentY = gamepad2.y;
        boolean currentB = gamepad2.b;
        boolean currentLB = gamepad2.left_bumper;
        boolean currentRB = gamepad2.right_bumper;

        // --- Adjust targetRPM ---
        if(currentLB && !lastLB) targetRPM = Math.max(500, targetRPM - 250);
        if(currentRB && !lastRB) targetRPM = Math.min(6000, targetRPM + 250);
        lastLB = currentLB;
        lastRB = currentRB;

        targetTPS = targetRPM / 60.0 * 28.0;

        // --- Toggle Intake / Servo ---
        if(currentY && !lastY) intakeOn = !intakeOn;
        if(currentA && !lastA) {
            servoOpen = !servoOpen;
            gateServo.setPosition(servoOpen ? openPosition : closePosition);
        }

        // --- Toggle Shooter ---
        if(currentX && !lastX) shooterOn = !shooterOn;

        // --- B Sequence ---
        if(currentB && !lastB && bState == BState.DONE){
            bState = BState.OPEN_SERVO;
            bSequenceStart = System.currentTimeMillis();
            gateServo.setPosition(openPosition);
        }

        // --- Update B sequence and intake ---
        if(bState != BState.DONE){
            long elapsed = System.currentTimeMillis() - bSequenceStart;
            switch(bState){
                case OPEN_SERVO:
                    if(elapsed >= 200){ // 0.2 секунды
                        Intake.setPower(1.0);
                        bSequenceStart = System.currentTimeMillis();
                        bState = BState.INTAKE;
                    }
                    break;
                case INTAKE:
                    if(elapsed >= 500){ // 0.5 секунды
                        Intake.setPower(0);
                        gateServo.setPosition(closePosition);
                        bState = BState.DONE;
                    }
                    break;
                default:
                    break;
            }
        } else {
            // если B-последовательность не активна, управляем интейком вручную
            Intake.setPower(intakeOn ? 1.0 : 0.0);
        }

        // --- Shooter PID ---
        double currentRPM = Outtake.getVelocity()/28.0*60.0;
        if(shooterOn) {
            if(targetRPM - currentRPM > dropThreshold) boostStart = System.currentTimeMillis();
            boolean boosting = (System.currentTimeMillis() - boostStart) < boostTime;

            double velocityToSet = targetTPS * (boosting ? 1.0 + boostPower : 1.0);
            Outtake.setVelocity(velocityToSet);
        } else {
            Outtake.setPower(0);
        }

        // --- Update last buttons ---
        lastX = currentX;
        lastA = currentA;
        lastY = currentY;
        lastB = currentB;

        // --- Telemetry ---
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Intake", Intake.getPower());
        telemetry.addData("Servo Pos", gateServo.getPosition());
        telemetry.addData("B Sequence Active", bState != BState.DONE);
        telemetry.update();
    }

    private void driveFieldRelative(double forward, double right, double rotate){
        double theta = Math.atan2(forward,right);
        double r = Math.hypot(right, forward);
        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        drive(r*Math.sin(theta), r*Math.cos(theta), rotate);
    }

    private void drive(double forward, double right, double rotate){
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double br = forward + right - rotate;
        double bl = forward - right + rotate;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(br), Math.abs(bl)))));
        frontLeftDrive.setPower(fl/max);
        frontRightDrive.setPower(fr/max);
        backLeftDrive.setPower(bl/max);
        backRightDrive.setPower(br/max);
    }


}
