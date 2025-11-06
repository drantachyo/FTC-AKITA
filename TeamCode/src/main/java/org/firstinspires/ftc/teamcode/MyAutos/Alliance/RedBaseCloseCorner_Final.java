package org.firstinspires.ftc.teamcode.MyAutos.Alliance;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

@Autonomous(name = "RBCCF", group = "Auto")
public class RedBaseCloseCorner_Final extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        boolean isBlue = false;
        double side = isBlue ? 1 : -1;

        Pose2d startPose = new Pose2d(61, -14 * side, Math.toRadians(-180) * side);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        double shooterStartPower = 0.6;

        waitForStart();
        if (isStopRequested()) return;

        shooter.setPower(shooterStartPower);

        // --- Первый подъезд к обелиску ---
        Action path = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(-138) * side)
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // --- Подъезд к первым шарам ---
        path = drive.actionBuilder(new Pose2d(-19, -16 * side, Math.toRadians(-138) * side))
                .strafeToLinearHeading(new Vector2d(34.5, -15 * side), Math.toRadians(-90) * side)
                .setTangent(Math.toRadians(270) * side)
                .lineToYConstantHeading(-55 * side)
                .lineToYConstantHeading(-16 * side)
                .build();
        Actions.runBlocking(path);

        // --- Возврат к обелиску ---
        path = drive.actionBuilder(new Pose2d(34.5, -16 * side, Math.toRadians(-90) * side))
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(-138) * side)
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // --- Выезд ко вторым шарам ---
        path = drive.actionBuilder(new Pose2d(-19, -16 * side, Math.toRadians(-138) * side))
                .strafeToLinearHeading(new Vector2d(30, -16 * side), Math.toRadians(-90) * side)
                .strafeToLinearHeading(new Vector2d(44, -60 * side), Math.toRadians(0) * side)
                .setTangent(Math.toRadians(0))
                .lineToXConstantHeading(60 * side)
                .build();
        Actions.runBlocking(path);

        // --- Финальный подъезд к обелиску ---
        path = drive.actionBuilder(new Pose2d(60, -60 * side, Math.toRadians(0) * side))
                .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(-138) * side)
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // --- Финальный отъезд для паркинга ---
        path = drive.actionBuilder(new Pose2d(-19, -16 * side, Math.toRadians(-138) * side))
                .strafeToConstantHeading(new Vector2d(30, -16 * side))
                .build();
        Actions.runBlocking(path);
    }

    private void fireBpulse(Intake intake, Shooter shooter, double startPower) throws InterruptedException {
        final double BOOST = Math.min(startPower + 0.2, 1.0);

        // 1) открыть гейт
        shooter.openGate();

        // 2) подать первый мяч
        intake.setPower(1.0);
        sleep(250); // можно подстроить по скорости подачи
        intake.setPower(0.0);

        // 3) закрыть гейт
        shooter.closeGate();
        sleep(150);

        // 4) буст мощности
        shooter.setPower(BOOST);
        sleep(300);

        // 5) открыть гейт (выстрел)
        shooter.openGate();
        sleep(200);

        // 6) подать второй мяч
        intake.setPower(1.0);
        sleep(400);
        intake.setPower(0.0);

        // 7) вернуть всё
        shooter.setPower(startPower);
        shooter.closeGate();
    }


}
