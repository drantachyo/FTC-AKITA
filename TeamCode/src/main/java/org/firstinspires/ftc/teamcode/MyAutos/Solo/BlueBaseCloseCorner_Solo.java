package org.firstinspires.ftc.teamcode.MyAutos.Solo;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Autonomous(name = "BBCCS", group = "Auto")
public class BlueBaseCloseCorner_Solo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(62, -14, Math.toRadians(-180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        double shooterStartPower = 0.6;

        waitForStart();
        if (isStopRequested()) return;

        shooter.setPower(shooterStartPower);

        // === 1. Первый выезд к обелиску ===
        Action toObelisk1 = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-13, -16), Math.toRadians(-135))
                .build();
        Actions.runBlocking(toObelisk1);
        fireBpulse(intake, shooter, shooterStartPower);

        // === 2. Первый забег за шарами ===
        intake.setPower(1.0);
        Action toBalls1 = drive.actionBuilder(new Pose2d(-13, -16, Math.toRadians(-135)))
                .strafeToConstantHeading(new Vector2d(34.5, -16))
                .strafeToLinearHeading(new Vector2d(34.5, -13), Math.toRadians(-90))
                .setTangent(Math.toRadians(270))
                .lineToYConstantHeading(-50)
                .lineToYConstantHeading(-16)
                .build();
        Actions.runBlocking(toBalls1);
        intake.setPower(0.0);

        // === 3. Второй выезд к обелиску ===
        Action toObelisk2 = drive.actionBuilder(new Pose2d(34.5, -16, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-13, -16), Math.toRadians(-135))
                .build();
        Actions.runBlocking(toObelisk2);
        fireBpulse(intake, shooter, shooterStartPower);

        // === 4. Второй забег за шарами ===
        intake.setPower(1.0);
        Action toBalls2 = drive.actionBuilder(new Pose2d(-13, -16, Math.toRadians(-135)))
                .strafeToLinearHeading(new Vector2d(11.6, -13), Math.toRadians(-90))
                .setTangent(Math.toRadians(270))
                .lineToYConstantHeading(-50)
                .lineToYConstantHeading(-16)
                .build();
        Actions.runBlocking(toBalls2);
        intake.setPower(0.0);

        // === 5. Третий выезд к обелиску ===
        Action toObelisk3 = drive.actionBuilder(new Pose2d(11.6, -16, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-13, -16), Math.toRadians(-135))
                .build();
        Actions.runBlocking(toObelisk3);
        fireBpulse(intake, shooter, shooterStartPower);

        // === 6. Третий забег за шарами ===
        intake.setPower(1.0);
        Action toBalls3 = drive.actionBuilder(new Pose2d(-13, -16, Math.toRadians(-135)))
                .strafeToConstantHeading(new Vector2d(-12, -16))
                .strafeToLinearHeading(new Vector2d(-12, -13), Math.toRadians(-90))
                .lineToYConstantHeading(-50)
                .lineToYConstantHeading(-16)
                .build();
        Actions.runBlocking(toBalls3);
        intake.setPower(0.0);

        // === 7. Четвёртый выезд к обелиску ===
        Action toObelisk4 = drive.actionBuilder(new Pose2d(-12, -16, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-12, -13), Math.toRadians(-135))
                .build();
        Actions.runBlocking(toObelisk4);
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
