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

@Autonomous(name = "RedBaseCloseCorner_Final", group = "Auto")
public class RedBaseCloseCorner_Final extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Starting pose for red base (mirrored Y)
        Pose2d startPose = new Pose2d(62, 14, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        double shooterStartPower = 0.6;

        waitForStart();
        if (isStopRequested()) return;

        // Keep shooter spinning
        shooter.setPower(shooterStartPower);

        // --- First shot ---
        Action path = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(8, 16), Math.toRadians(147))
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // --- First pickup run ---
        intake.setPower(1.0);
        path = drive.actionBuilder(new Pose2d(8.1, 16, Math.toRadians(148)))
                .strafeToConstantHeading(new Vector2d(34.5, 16))
                .strafeToLinearHeading(new Vector2d(34.5, 13), Math.toRadians(90))
                .setTangent(Math.toRadians(-270))
                .lineToYConstantHeading(55)
                .lineToYConstantHeading(10)
                .build();
        Actions.runBlocking(path);
        intake.setPower(0.0);

        // --- Second shot ---
        path = drive.actionBuilder(new Pose2d(34.5, 10, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(8, 16), Math.toRadians(143))
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);

        // --- Second pickup run ---
        intake.setPower(1.0);
        path = drive.actionBuilder(new Pose2d(8, 16, Math.toRadians(143)))
                .lineToXLinearHeading(30, Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(44, 60, Math.toRadians(0)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(44, 60), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .lineToXConstantHeading(59)
                .strafeToConstantHeading(new Vector2d(35, 30))
                .build();
        Actions.runBlocking(path);
        intake.setPower(0.0);

        // --- Final shot ---
        path = drive.actionBuilder(new Pose2d(35, 30, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(8, 16), Math.toRadians(143))
                .build();
        Actions.runBlocking(path);
        fireBpulse(intake, shooter, shooterStartPower);
    }

    private void fireBpulse(Intake intake, Shooter shooter, double startPower) throws InterruptedException {
        shooter.closeGate();

        // Stage 1: start intake, keep shooter steady
        intake.setPower(1.0);
        shooter.setPower(startPower);
        sleep(150);

        // Stage 2: boost shooter for firing
        intake.setPower(0.0);
        shooter.setPower(Math.min(startPower + 0.2, 1.0));
        sleep(650);

        // Stage 3: restore power, pull next ball
        intake.setPower(1.0);
        shooter.setPower(startPower);
        sleep(500);

        // Finish
        shooter.openGate();
        intake.setPower(0.0);
    }
}
