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

@Autonomous(name = "BlueBaseFarCorner", group = "Auto")
public class BlueBaseFarCorner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-50, -50, Math.toRadians(234));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
//        Shooter shooter = new Shooter(hardwareMap);
//        Intake intake = new Intake(hardwareMap);

        waitForStart();

        Action path = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(229))
//                .stopAndAdd(intake.runIntake())
//                .stopAndAdd(shooter.runShooter())
                .strafeToLinearHeading(new Vector2d(-19.1, -16), Math.toRadians(229))
//                .stopAndAdd(shooter.feed())

                .strafeToLinearHeading(new Vector2d(-9.5, -27), Math.toRadians(271))
//                .stopAndAdd(shooter.stopShooter())
//                .stopAndAdd(shooter.resetFeeder())
                .strafeToLinearHeading(new Vector2d(-9.5, -50),Math.toRadians(271))

                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(229))
//                .stopAndAdd(shooter.runShooter())
                .strafeToLinearHeading(new Vector2d(-19.1, -16), Math.toRadians(229))
//                .stopAndAdd(shooter.feed())



                .strafeToLinearHeading(new Vector2d(14, -27), Math.toRadians(271))
//                .stopAndAdd(shooter.stopShooter())
//                .stopAndAdd(shooter.resetFeeder())
                .strafeToLinearHeading(new Vector2d(14, -50),Math.toRadians(271))
                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(229))
//                .stopAndAdd(shooter.runShooter())
                .strafeToLinearHeading(new Vector2d(-19.1, -16), Math.toRadians(229))
//                .stopAndAdd(shooter.feed())
                .build();


        Actions.runBlocking(path);
    }
}
