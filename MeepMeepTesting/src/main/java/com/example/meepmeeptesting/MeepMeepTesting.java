package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 55, Math.toRadians(450), Math.toRadians(450), 15)
                .build();


                myBot.runAction(
                        myBot.getDrive().actionBuilder(new Pose2d(-50, -50, Math.toRadians(234)))
                                .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(229))
//                .stopAndAdd(intake.runIntake())
//                .stopAndAdd(shooter.runShooter())
                        .strafeToLinearHeading(new Vector2d(-19.1, -16), Math.toRadians(229))
//                .stopAndAdd(shooter.feed())

                        .strafeToLinearHeading(new Vector2d(-11.8, -27), Math.toRadians(270))
//                .stopAndAdd(shooter.stopShooter())
//                .stopAndAdd(shooter.resetFeeder())
                        .strafeToLinearHeading(new Vector2d(-11.8, -50),Math.toRadians(270))

                        .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(229))
//                .stopAndAdd(shooter.runShooter())
                        .strafeToLinearHeading(new Vector2d(-19.1, -16), Math.toRadians(229))
//                .stopAndAdd(shooter.feed())



                        .strafeToLinearHeading(new Vector2d(11.6, -27), Math.toRadians(270))
//                .stopAndAdd(shooter.stopShooter())
//                .stopAndAdd(shooter.resetFeeder())
                        .strafeToLinearHeading(new Vector2d(11.6, -50),Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(229))
//                .stopAndAdd(shooter.runShooter())
                        .strafeToLinearHeading(new Vector2d(-19.1, -16), Math.toRadians(229))
//                .stopAndAdd(shooter.feed())
                        .build());







        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();




    }
}