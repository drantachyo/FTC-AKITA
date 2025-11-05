package org.firstinspires.ftc.teamcode.MyAutos.NoINTAKE;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Autonomous(name = "BlueBaseCloseCorner_NoIntake", group = "Auto")
public class BlueBaseCloseCorner_NoIntake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Синяя база
        Pose2d startPose = new Pose2d(62, -14, Math.toRadians(-180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);

        double shooterStartPower = 0.6; // как в TeleOp

        waitForStart();

        // Включаем постоянную раскрутку шутера
        shooter.setPower(shooterStartPower);

        Action path = drive.actionBuilder(startPose)
                // подъезд к обелиску для первого выстрела
                .strafeToLinearHeading(new Vector2d(8, -16), Math.toRadians(-147))
                .build();

        Actions.runBlocking(path);

        // автоматический B-пульс перед обелиском
        fireBpulse(shooter, shooterStartPower);

        // движение к первым шарам (теперь без интейка)
        path = drive.actionBuilder(new Pose2d(8.1, -16, Math.toRadians(-148)))
                .strafeToConstantHeading(new Vector2d(34.5, -16))
                .strafeToLinearHeading(new Vector2d(34.5, -13), Math.toRadians(-90))
                .setTangent(Math.toRadians(270))
                .lineToYConstantHeading(-55)
                .lineToYConstantHeading(-10)
                .build();
        Actions.runBlocking(path);

        // возврат к обелиску и второй B-пульс
        path = drive.actionBuilder(new Pose2d(34.5, -10, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(8, -16), Math.toRadians(-143))
                .build();
        Actions.runBlocking(path);

        fireBpulse(shooter, shooterStartPower);

        // выезд ко вторым шарам
        path = drive.actionBuilder(new Pose2d(8, -16, Math.toRadians(-143)))
                .lineToXLinearHeading(30, Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(44, -60, Math.toRadians(0)), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(44, -60), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .lineToXConstantHeading(59)
                .strafeToConstantHeading(new Vector2d(35, -30))
                .build();
        Actions.runBlocking(path);

        // финальный подъезд к обелиску и выстрел
        path = drive.actionBuilder(new Pose2d(35, -30, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(8, -16), Math.toRadians(-143))
                .build();
        Actions.runBlocking(path);

        fireBpulse(shooter, shooterStartPower);
    }

    private void fireBpulse(Shooter shooter, double startPower) throws InterruptedException {
        sleep(4000);
        shooter.closeGate();
        //intake
        sleep(500);
        shooter.setPower(startPower);
        shooter.openGate();
        shooter.setPower(Math.min(startPower + 0.2, 1.0));
        sleep(1000);
        shooter.closeGate();
        sleep(500);
        shooter.setPower(startPower);
        shooter.openGate();
    }
}
