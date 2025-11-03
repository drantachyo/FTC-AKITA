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

@Autonomous(name = "RedBaseFarCorner", group = "Auto")
public class RedBaseFarCorner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-50, 50, Math.toRadians(-234));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        boolean isBlue = false; // красная сторона
        double side = isBlue ? -1 : 1;

        double shooterStartPower = 0.65; // как в TeleOp

        waitForStart();

        // Включаем постоянную раскрутку шутера
        shooter.setPower(shooterStartPower);

        // подъезд к первому обелиску
        Action path = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(229) * side)
                .strafeToLinearHeading(new Vector2d(-19.1, -16 * side), Math.toRadians(229) * side)
                .build();
        Actions.runBlocking(path);

        // первый автоматический B-пульс
        fireBpulse(intake, shooter, shooterStartPower);

        // подъезд к первым шарам с включённым интейком
        intake.setPower(1.0);
        path = drive.actionBuilder(new Pose2d(-19.1, -16 * side, Math.toRadians(229) * side))
                .strafeToLinearHeading(new Vector2d(-9.5, -27 * side), Math.toRadians(271) * side)
                .strafeToLinearHeading(new Vector2d(-9.5, -50 * side), Math.toRadians(271) * side)
                .build();
        Actions.runBlocking(path);
        intake.setPower(0.0);

        // возврат к обелиску и второй B-пульс
        path = drive.actionBuilder(new Pose2d(-9.5, -50 * side, Math.toRadians(271) * side))
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(229) * side)
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // выезд ко вторым шарам с интейком
        intake.setPower(1.0);
        path = drive.actionBuilder(new Pose2d(-19, -16 * side, Math.toRadians(229) * side))
                .strafeToLinearHeading(new Vector2d(-19.1, -16 * side), Math.toRadians(229) * side)
                .strafeToLinearHeading(new Vector2d(14, -27 * side), Math.toRadians(271) * side)
                .strafeToLinearHeading(new Vector2d(14, -50 * side), Math.toRadians(271) * side)
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(229) * side)
                .strafeToLinearHeading(new Vector2d(-19.1, -16 * side), Math.toRadians(229) * side)
                .strafeToConstantHeading(new Vector2d(14,-14 * side))
                .build();
        Actions.runBlocking(path);
        intake.setPower(0.0);

        // финальный подъезд к обелиску и выстрел
        path = drive.actionBuilder(new Pose2d(14, -14 * side, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(229) * side)
                .build();
        Actions.runBlocking(path);

        fireBpulse(intake, shooter, shooterStartPower);
    }

    private void fireBpulse(Intake intake, Shooter shooter, double startPower) throws InterruptedException {
        // закрываем серво в начале импульса
        shooter.closeGate();

        // первый этап B-пульса
        intake.setPower(1.0);
        shooter.setPower(startPower);
        sleep(150);

        // второй этап
        intake.setPower(0.0);
        shooter.setPower(Math.min(startPower + 0.2, 1.0));
        sleep(650);

        // третий этап
        intake.setPower(1.0);
        shooter.setPower(startPower);
        sleep(500);

        // открываем серво в конце
        shooter.openGate();

        // выключаем интейк
        intake.setPower(0.0);
    }
}
