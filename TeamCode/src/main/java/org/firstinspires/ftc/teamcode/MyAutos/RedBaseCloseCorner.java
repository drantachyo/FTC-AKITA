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

@Autonomous(name = "BlueBaseCloseCorner_Mirrored", group = "Auto")
public class RedBaseCloseCorner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // зеркальная стартовая позиция (по оси X)
        Pose2d startPose = new Pose2d(62, 14, Math.toRadians(-180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        Action path = drive.actionBuilder(startPose)
                // выплюнуть шары загруженные
                .strafeToLinearHeading(new Vector2d(-19, 16), Math.toRadians(-229))
                .stopAndAdd(shooter.runShooter())
                .stopAndAdd(intake.runIntake())
                .strafeToLinearHeading(new Vector2d(-19.1, 16), Math.toRadians(-229))
                .stopAndAdd(shooter.feed())

                // подъехать к шарам 3
                .strafeToLinearHeading(new Vector2d(35, 30), Math.toRadians(-270))
                .stopAndAdd(shooter.stopShooter())
                .stopAndAdd(shooter.resetFeeder())
                .setTangent(Math.toRadians(-270))
                .lineToYConstantHeading(55)

                // выплюнуть шары 3
                .strafeToLinearHeading(new Vector2d(-19, 16), Math.toRadians(-229))
                .stopAndAdd(shooter.runShooter())
                .strafeToLinearHeading(new Vector2d(-19.1, 16), Math.toRadians(-229))
                .stopAndAdd(shooter.feed())
                .build();

        Actions.runBlocking(path);
    }
}
