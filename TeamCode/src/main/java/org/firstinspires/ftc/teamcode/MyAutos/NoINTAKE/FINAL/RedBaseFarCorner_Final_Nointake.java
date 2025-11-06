package org.firstinspires.ftc.teamcode.MyAutos.NoINTAKE.FINAL;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Autonomous(name = "RBFCFN", group = "Auto")
public class RedBaseFarCorner_Final_Nointake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isRed = true;
        double side = isRed ? -1 : 1;

        Pose2d startPose = new Pose2d(-50, -50 * side, Math.toRadians(234 * side));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);

        double shooterStartPower = 0.47;

        waitForStart();
        if (isStopRequested()) return;

        shooter.setPower(shooterStartPower);

        // --- Первый подъезд к обелиску ---
        Action path = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(230 * side))
                .build();
        Actions.runBlocking(path);
        fireBpulse(shooter, shooterStartPower);

        // --- Первый сбор шаров ---
        path = drive.actionBuilder(new Pose2d(-19, -16 * side, Math.toRadians(230 * side)))
                .strafeToLinearHeading(new Vector2d(-11.5, -16 * side), Math.toRadians(270 * side))
                .strafeToLinearHeading(new Vector2d(-11.5, -50 * side), Math.toRadians(270 * side))
                .build();
        Actions.runBlocking(path);

        // --- Второй подъезд к обелиску ---
        path = drive.actionBuilder(new Pose2d(-11.5, -50 * side, Math.toRadians(270 * side)))
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(230 * side))
                .build();
        Actions.runBlocking(path);
        fireBpulse(shooter, shooterStartPower);

        // --- Второй сбор шаров ---
        path = drive.actionBuilder(new Pose2d(-19, -16 * side, Math.toRadians(230 * side)))
                .strafeToLinearHeading(new Vector2d(11.5, -16 * side), Math.toRadians(270 * side))
                .strafeToLinearHeading(new Vector2d(11.5, -50 * side), Math.toRadians(270 * side))
                .build();
        Actions.runBlocking(path);

        // --- Третий подъезд к обелиску ---
        path = drive.actionBuilder(new Pose2d(11.5, -50 * side, Math.toRadians(270 * side)))
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(230 * side))
                .build();
        Actions.runBlocking(path);
        fireBpulse(shooter, shooterStartPower);

        // --- Финальная парковка ---
        path = drive.actionBuilder(new Pose2d(-19, -16 * side, Math.toRadians(230 * side)))
                .strafeToConstantHeading(new Vector2d(14, -14 * side))
                .build();
        Actions.runBlocking(path);
    }

    private void fireBpulse(Shooter shooter, double startPower) throws InterruptedException {
        final double BOOST = Math.min(startPower + 0.2, 1.0);

        // 1) открыть гейт (разрешаем мячу попасть в шутер)
        shooter.openGate();

        // 2) короткий подача, чтобы мяч встал в позицию
        sleep(600);


        // 3) закрыть гейт — фиксируем мяч внутри шутер
        shooter.closeGate();

        // 4) поднять мощность для компенсации потери импульса
        shooter.setPower(BOOST);
        sleep(300); // даём немного времени на повышение оборотов

        // 5) открыть гейт — производим выстрел
        shooter.openGate();

        // 6) подать второй мяч (проталкиваем следующий)
        sleep(600);

        // 7) вернуть всё в исходное состояние: мощность и закрыть гейт
        shooter.setPower(startPower);
        shooter.closeGate();
    }
}
