package org.firstinspires.ftc.teamcode.MyAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Autonomous(name = "BBFC_Mangystau_PID2_MEPMEEP_StrafeTurn", group = "Auto")
public class BlueBaseFarCornerStrafeTurn extends LinearOpMode {

    double kP = 33.0, kI = 0, kD = 2.0, kF = 14.0;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(-51, -49, Math.toRadians(-125));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        DcMotorEx shooterMotor = shooter.getMotor();
        shooterMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );

        waitForStart();

        setShooterRPM(shooter, 1000);
        intake.collect();
        sleep(150);
        intake.stop();

        if (isStopRequested()) return;

        // === 1. Первый выезд к обелиску ===
        setShooterRPM(shooter, 2800);
        waitForShooterStable(shooter);
        Action toObelisk1 = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-12, -16), Math.toRadians(-135))
                .build();
        Actions.runBlocking(toObelisk1);
        fireSequencePID(intake);

        // === 2. Первый забег за шарами ===
        setShooterRPM(shooter, 1200);
        intake.collect();
        // вместо turn делаем микро-смещение по Y с новым heading
        Action toBalls1 = drive.actionBuilder(new Pose2d(-12, -16, Math.toRadians(-135)))
                .strafeToLinearHeading(new Vector2d(-12, -17), Math.toRadians(45))
                .strafeToConstantHeading(new Vector2d(-12, -53))
                .strafeToConstantHeading(new Vector2d(-3, -40))
                .strafeToConstantHeading(new Vector2d(-3, -53))
                .build();
        Actions.runBlocking(toBalls1);
        intake.stop();

        // === 3. Второй выезд к обелиску ===
        setShooterRPM(shooter, 2800);
        waitForShooterStable(shooter);
        Action toObelisk2 = drive.actionBuilder(new Pose2d(-3, -53, Math.toRadians(-54)))
                .strafeToLinearHeading(new Vector2d(-12, -16), Math.toRadians(-135))
                .build();
        Actions.runBlocking(toObelisk2);
        fireSequencePID(intake);

        // === 4. Второй забег за шарами ===
        setShooterRPM(shooter, 1200);
        intake.collect();
        Action toBalls2 = drive.actionBuilder(new Pose2d(-12, -16, Math.toRadians(-135)))
                .strafeToLinearHeading(new Vector2d(11.5, -17), Math.toRadians(-90))
                .setTangent(Math.toRadians(270))
                .lineToYConstantHeading(-60)
                .lineToYConstantHeading(-50)
                .build();
        Actions.runBlocking(toBalls2);
        intake.stop();

        // === 5. Третий выезд к обелиску ===
        setShooterRPM(shooter, 2800);
        waitForShooterStable(shooter);
        Action toObelisk3 = drive.actionBuilder(new Pose2d(11.5, -50, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-12, -16), Math.toRadians(-135))
                .build();
        Actions.runBlocking(toObelisk3);
        fireSequencePID(intake);

        // === 6. Третий забег за шарами ===
        setShooterRPM(shooter, 1200);
        intake.collect();
        Action toBalls3 = drive.actionBuilder(new Pose2d(-12, -16, Math.toRadians(-135)))
                .strafeToLinearHeading(new Vector2d(34.5, -15), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .lineToYConstantHeading(-60)
                .build();
        Actions.runBlocking(toBalls3);
        intake.stop();

        // === 7. Четвёртый выезд к обелиску ===
        setShooterRPM(shooter, 3700);
        waitForShooterStable(shooter);
        Action toObelisk4 = drive.actionBuilder(new Pose2d(34.5, -60, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(50, -14), Math.toRadians(-150))
                .build();
        Actions.runBlocking(toObelisk4);
        fireSequencePID(intake);
    }

    private void fireSequencePID(Intake intake) throws InterruptedException {
        intake.feed();
        Thread.sleep(1400);
        intake.stop();
        Thread.sleep(200);
    }

    private void setShooterRPM(Shooter shooter, double rpm) {
        shooter.setTargetRPM(rpm);
    }

    private void waitForShooterStable(Shooter shooter) throws InterruptedException {
        while (!shooter.isStable() && !isStopRequested()) {
            shooter.update();
            sleep(10);
        }
    }


}
