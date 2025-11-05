package org.firstinspires.ftc.teamcode.MyAutos.NoINTAKE;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Autonomous(name = "RedBaseFarCorner_NoIntake", group = "Auto")
public class RedBaseFarCorner_NoIntake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-50, 50, Math.toRadians(-234));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);

        boolean isBlue = false; // красная сторона
        double side = isBlue ? -1 : 1;

        double shooterStartPower = 0.58; // как в TeleOp

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
        fireBpulse(shooter, shooterStartPower);

        // движение к первым шарам (без интейка)
        path = drive.actionBuilder(new Pose2d(-19.1, -16 * side, Math.toRadians(229) * side))
                .strafeToLinearHeading(new Vector2d(-9.5, -27 * side), Math.toRadians(271) * side)
                .strafeToLinearHeading(new Vector2d(-9.5, -50 * side), Math.toRadians(271) * side)
                .build();
        Actions.runBlocking(path);

        // возврат к обелиску и второй B-пульс
        path = drive.actionBuilder(new Pose2d(-9.5, -50 * side, Math.toRadians(271) * side))
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(229) * side)
                .build();
        Actions.runBlocking(path);
        fireBpulse(shooter, shooterStartPower);

        // выезд ко вторым шарам (без интейка)
        path = drive.actionBuilder(new Pose2d(-19, -16 * side, Math.toRadians(229) * side))
                .strafeToLinearHeading(new Vector2d(-19.1, -16 * side), Math.toRadians(229) * side)
                .strafeToLinearHeading(new Vector2d(14, -27 * side), Math.toRadians(271) * side)
                .strafeToLinearHeading(new Vector2d(14, -50 * side), Math.toRadians(271) * side)
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(229) * side)
                .strafeToLinearHeading(new Vector2d(-19.1, -16 * side), Math.toRadians(229) * side)
                .strafeToConstantHeading(new Vector2d(14, -14 * side))
                .build();
        Actions.runBlocking(path);

        // финальный подъезд к обелиску и выстрел
        path = drive.actionBuilder(new Pose2d(14, -14 * side, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(229) * side)
                .build();
        Actions.runBlocking(path);

        fireBpulse(shooter, shooterStartPower);
    }

    private void fireBpulse(Shooter shooter, double startPower) throws InterruptedException {
        sleep(3000);
        shooter.closeGate();

        // "B-пульс" — последовательность изменения мощности
        shooter.setPower(startPower);
        sleep(150);

        shooter.setPower(Math.min(startPower + 0.2, 1.0));
        sleep(650);

        shooter.setPower(startPower);
        sleep(500);

        shooter.openGate();
    }
}
