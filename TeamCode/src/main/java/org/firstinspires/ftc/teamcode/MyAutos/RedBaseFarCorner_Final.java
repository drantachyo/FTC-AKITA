package org.firstinspires.ftc.teamcode.MyAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

@Autonomous(name = "RedBaseFarCorner_Final", group = "Auto")
public class RedBaseFarCorner_Final extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // зеркальная стартовая позиция
        Pose2d startPose = new Pose2d(-50, 50, Math.toRadians(-234));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        double shooterStartPower = 0.47;

        waitForStart();
        if (isStopRequested()) return;

        // включаем постоянную раскрутку
        shooter.setPower(shooterStartPower);

        // --- первый выстрел ---
        Action path = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-19, 16), Math.toRadians(-232))
                .strafeToLinearHeading(new Vector2d(-19.1, 16), Math.toRadians(-232))
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // --- сбор первых шаров ---
        intake.setPower(1.0);
        path = drive.actionBuilder(new Pose2d(-19.1, 16, Math.toRadians(-232)))
                .strafeToLinearHeading(new Vector2d(-9, 27), Math.toRadians(-271))
                .strafeToLinearHeading(new Vector2d(-9, 50), Math.toRadians(-271))
                .build();
        Actions.runBlocking(path);
        intake.setPower(0.0);

        // --- второй выстрел ---
        path = drive.actionBuilder(new Pose2d(-9, 50, Math.toRadians(-232)))
                .strafeToLinearHeading(new Vector2d(-19, 16), Math.toRadians(-229))
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // --- сбор вторых шаров ---
        intake.setPower(1.0);
        path = drive.actionBuilder(new Pose2d(-19, 16, Math.toRadians(-232)))
                .strafeToLinearHeading(new Vector2d(-19.1, 16), Math.toRadians(-232))
                .strafeToLinearHeading(new Vector2d(14.5, 27), Math.toRadians(-271))
                .strafeToLinearHeading(new Vector2d(14.5, 50), Math.toRadians(-271))
                .strafeToLinearHeading(new Vector2d(-19, 16), Math.toRadians(-232))
                .strafeToLinearHeading(new Vector2d(-19.1, 16), Math.toRadians(-232))
                .build();
        Actions.runBlocking(path);
        intake.setPower(0.0);

        // --- третий выстрел ---
        fireBpulse(intake, shooter, shooterStartPower);

        // --- финальный паркинг ---
        path = drive.actionBuilder(new Pose2d(-19.1, 16, Math.toRadians(-232)))
                .strafeToConstantHeading(new Vector2d(14, 14))
                .build();
        Actions.runBlocking(path);
    }

    private void fireBpulse(Intake intake, Shooter shooter, double startPower) throws InterruptedException {
        shooter.closeGate();

        intake.setPower(1.0);
        shooter.setPower(startPower);
        sleep(150);

        intake.setPower(0.0);
        shooter.setPower(Math.min(startPower + 0.2, 1.0));
        sleep(650);

        intake.setPower(1.0);
        shooter.setPower(startPower);
        sleep(500);

        shooter.openGate();
        intake.setPower(0.0);
    }
}
