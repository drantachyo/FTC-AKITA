package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Autonomous(name = "Auton Diagonal", group = "Auto")
public class Auton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(57, -33, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(hardwareMap);

        waitForStart();

        Action path = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(25, -21))
                .splineToLinearHeading(new Pose2d(-12, -27, Math.toRadians(270)), Math.toRadians(250))
                .setTangent(Math.toRadians(270))
                .lineToYConstantHeading(-50)
                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(225))

                .waitSeconds(0.5)


                .splineToLinearHeading(new Pose2d(12, -27, Math.toRadians(270)), Math.toRadians(225))
                .setTangent(Math.toRadians(270))
                .lineToYConstantHeading(-50)
                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(225))

                .waitSeconds(0.5)


                .strafeToLinearHeading(new Vector2d(35, -27), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToYConstantHeading(-50)
                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(225))

                .waitSeconds(0.5)

                .build();

        Actions.runBlocking(path);
    }
}
