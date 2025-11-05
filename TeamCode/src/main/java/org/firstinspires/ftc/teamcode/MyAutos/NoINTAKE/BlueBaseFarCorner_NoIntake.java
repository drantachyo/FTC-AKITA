package org.firstinspires.ftc.teamcode.MyAutos.NoINTAKE;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Autonomous(name = "BlueBaseFarCorner_NoIntake", group = "Auto")
public class BlueBaseFarCorner_NoIntake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-50, -50, Math.toRadians(234));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);

        double shooterStartPower = 0.47; // как в TeleOp

        waitForStart();

        // Включаем постоянную раскрутку шутера
        shooter.setPower(shooterStartPower);

        // подъезд к первому обелиску
        Action path = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(232))
                .strafeToLinearHeading(new Vector2d(-19.1, -16), Math.toRadians(232))
                .build();
        Actions.runBlocking(path);

        // первый автоматический B-пульс
        fireBpulse(shooter, shooterStartPower);

        // движение к первым шарам (без интейка)
        path = drive.actionBuilder(new Pose2d(-19.1, -16, Math.toRadians(232)))
                .strafeToLinearHeading(new Vector2d(-9, -27), Math.toRadians(271))
                .strafeToLinearHeading(new Vector2d(-9, -50), Math.toRadians(271))
                .build();
        Actions.runBlocking(path);

        // возврат к обелиску и второй B-пульс
        path = drive.actionBuilder(new Pose2d(-9, -50, Math.toRadians(232)))
                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(229))
                .build();
        Actions.runBlocking(path);
        fireBpulse(shooter, shooterStartPower);

        // выезд ко вторым шарам (без интейка)
        path = drive.actionBuilder(new Pose2d(-19, -16, Math.toRadians(232)))
                .strafeToLinearHeading(new Vector2d(-19.1, -16), Math.toRadians(232))
                .strafeToLinearHeading(new Vector2d(14.5, -27), Math.toRadians(271))
                .strafeToLinearHeading(new Vector2d(14.5, -50), Math.toRadians(271))
                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(232))
                .strafeToLinearHeading(new Vector2d(-19.1, -16), Math.toRadians(232))

                .build();
        Actions.runBlocking(path);
        fireBpulse(shooter, shooterStartPower);

        // финальный подъезд к обелиску и выстрел
        path = drive.actionBuilder(new Pose2d(-19.1, -16, Math.toRadians(232)))
                 .strafeToConstantHeading(new Vector2d(14, -14))
                .build();
        Actions.runBlocking(path);


    }

    private void fireBpulse(Shooter shooter, double startPower) throws InterruptedException {
        sleep(2000);
        shooter.closeGate();

        // "B-пульс" — тройной импульс мощности шутера
        shooter.setPower(startPower);
        sleep(1000);

        shooter.setPower(Math.min(startPower + 0.2, 1.0));
        sleep(650);

        shooter.setPower(startPower);
        sleep(500);

        shooter.openGate();
    }
}
