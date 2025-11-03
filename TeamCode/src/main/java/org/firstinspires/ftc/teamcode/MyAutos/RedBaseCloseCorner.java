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

@Autonomous(name = "RedBaseCloseCorner", group = "Auto")
public class RedBaseCloseCorner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // зеркальная стартовая позиция (по оси X)
        Pose2d startPose = new Pose2d(62, 14, Math.toRadians(-180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        double shooterStartPower = 0.65;

        waitForStart();

        // постоянная раскрутка шутера
        shooter.setPower(shooterStartPower);

        // подъезд к первому обелиску и первый B-пульс
        Action path = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(8, 16), Math.toRadians(148))
                .strafeToLinearHeading(new Vector2d(8.1, 16), Math.toRadians(148))
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // движение к первым шарам с включённым интейком
        intake.setPower(1.0);
        path = drive.actionBuilder(new Pose2d(8.1, 16, Math.toRadians(148)))
                .splineToSplineHeading(new Pose2d(33, 35, Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(33, 35), Math.toRadians(90))
                .setTangent(Math.toRadians(-270))
                .lineToYConstantHeading(55)
                .lineToYConstantHeading(16)
                .build();
        Actions.runBlocking(path);
        intake.setPower(0.0);

        // возврат к обелиску и второй B-пульс
        path = drive.actionBuilder(new Pose2d(33, 16, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(8, 16), Math.toRadians(148))
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // выезд ко вторым шарам с интейком
        intake.setPower(1.0);
        path = drive.actionBuilder(new Pose2d(8, 16, Math.toRadians(148)))
                .lineToXLinearHeading(30, Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(44, 60, Math.toRadians(0)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(44, 60), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .lineToXConstantHeading(59)
                .strafeToConstantHeading(new Vector2d(35, 30))
                .build();
        Actions.runBlocking(path);
        intake.setPower(0.0);

        // финальный подъезд к обелиску и выстрел
        path = drive.actionBuilder(new Pose2d(35, 30, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(8, 16), Math.toRadians(148))
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);
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
