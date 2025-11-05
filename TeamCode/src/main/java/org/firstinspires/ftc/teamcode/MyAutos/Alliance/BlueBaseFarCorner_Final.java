package org.firstinspires.ftc.teamcode.MyAutos.Alliance;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

@Autonomous(name = "BlueBaseFarCorner_Full", group = "Auto")
public class BlueBaseFarCorner_Final extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-50, -50, Math.toRadians(234));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        double shooterStartPower = 0.47;

        waitForStart();
        if (isStopRequested()) return;

        shooter.setPower(shooterStartPower);

        // --- Первый подъезд к обелиску ---
        Action path = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(230))
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // --- Первый сбор шаров ---
        intake.setPower(1.0);
        path = drive.actionBuilder(new Pose2d(-19, -16, Math.toRadians(230)))
                .strafeToLinearHeading(new Vector2d(-11.5, -16), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-11.5, -50), Math.toRadians(270))
                .build();
        Actions.runBlocking(path);
        intake.setPower(0.0);

        // --- Второй подъезд к обелиску ---
        path = drive.actionBuilder(new Pose2d(-11.5, -50, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(230))
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // --- Второй сбор шаров ---
        intake.setPower(1.0);
        path = drive.actionBuilder(new Pose2d(-19, -16, Math.toRadians(230)))
                .strafeToLinearHeading(new Vector2d(11.5, -16), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(11.5, -50), Math.toRadians(270))
                .build();
        Actions.runBlocking(path);
        intake.setPower(0.0);

        // --- Третий подъезд к обелиску ---
        path = drive.actionBuilder(new Pose2d(11.5, -50, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(230))
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // --- Финальная парковка ---
        path = drive.actionBuilder(new Pose2d(-19, -16, Math.toRadians(230)))
                .strafeToConstantHeading(new Vector2d(14, -14))
                .build();
        Actions.runBlocking(path);
    }

    private void fireBpulse(Intake intake, Shooter shooter, double startPower) throws InterruptedException {
        shooter.closeGate();
        intake.setPower(0.0); // остановка интейка на выстрел
        shooter.setPower(startPower);
        sleep(150);
        shooter.setPower(Math.min(startPower + 0.2, 1.0));
        sleep(650);
        shooter.setPower(startPower);
        shooter.openGate();
    }
}
