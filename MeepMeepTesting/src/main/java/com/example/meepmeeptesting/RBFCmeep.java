package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RBFCmeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // true = синяя база
        boolean isBlue = true;
        double side = isBlue ? -1 : 1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 55, Math.toRadians(450), Math.toRadians(450), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(
                                new Pose2d(-50, -50 * side, Math.toRadians(234) * side))
                        .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(229) * side)
                        .strafeToLinearHeading(new Vector2d(-19.1, -16 * side), Math.toRadians(229) * side)
                        .strafeToLinearHeading(new Vector2d(-9.5, -27 * side), Math.toRadians(271) * side)
                        .strafeToLinearHeading(new Vector2d(-9.5, -50 * side), Math.toRadians(271) * side)
                        .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(229) * side)
                        .strafeToLinearHeading(new Vector2d(-19.1, -16 * side), Math.toRadians(229) * side)
                        .strafeToLinearHeading(new Vector2d(14, -27 * side), Math.toRadians(271) * side)
                        .strafeToLinearHeading(new Vector2d(14, -50 * side), Math.toRadians(271) * side)
                        .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(229) * side)
                        .strafeToLinearHeading(new Vector2d(-19.1, -16 * side), Math.toRadians(229) * side)
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
