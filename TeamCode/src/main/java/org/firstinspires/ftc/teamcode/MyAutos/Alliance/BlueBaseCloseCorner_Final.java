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

@Autonomous(name = "BlueBaseCloseCorner_Final", group = "Auto")
public class BlueBaseCloseCorner_Final extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        boolean isBlue = true;
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
                .strafeToLinearHeading(new Vector2d(-13, -16 * side), Math.toRadians(-135) * side)
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // --- Подъезд к первым шарам ---
        path = drive.actionBuilder(new Pose2d(-13, -16 * side, Math.toRadians(-135) * side))
                .strafeToLinearHeading(new Vector2d(34.5, -15 * side), Math.toRadians(-90) * side)
                .setTangent(Math.toRadians(270) * side)
                .lineToYConstantHeading(-55 * side)
                .lineToYConstantHeading(-16 * side)
                .build();
        intake.setPower(1.0);
        Actions.runBlocking(path);
        intake.setPower(0.0);

        // --- Возврат к обелиску ---
        path = drive.actionBuilder(new Pose2d(34.5, -16 * side, Math.toRadians(-90) * side))
                .strafeToLinearHeading(new Vector2d(-13, -16 * side), Math.toRadians(-135) * side)
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // --- Выезд ко вторым шарам ---
        path = drive.actionBuilder(new Pose2d(-13, -16 * side, Math.toRadians(-135) * side))
                .lineToXLinearHeading(30, Math.toRadians(-90) * side)
                .splineToSplineHeading(new Pose2d(44, -60 * side, Math.toRadians(0) * side), Math.toRadians(-90) * side)
                .strafeToLinearHeading(new Vector2d(44, -60 * side), Math.toRadians(0) * side)
                .setTangent(Math.toRadians(0))
                .lineToXConstantHeading(60)
                .build();
        intake.setPower(1.0);
        Actions.runBlocking(path);
        intake.setPower(0.0);

        // --- Финальный подъезд к обелиску ---
        path = drive.actionBuilder(new Pose2d(60, -60 * side, Math.toRadians(0) * side))
                .strafeToLinearHeading(new Vector2d(-13, -16 * side), Math.toRadians(-135) * side)
                .strafeToLinearHeading(new Vector2d(44, -16 * side), Math.toRadians(-180))
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
        shooter.setPower(startPower);
        shooter.openGate();
    }
}
