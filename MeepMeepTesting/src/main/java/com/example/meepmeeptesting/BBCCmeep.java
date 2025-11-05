package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BBCCmeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // true = синяя база
        boolean isBlue = true;
        double side = isBlue ? -1 : 1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 55, Math.toRadians(360), Math.toRadians(360), 15)
                .build();

        myBot.runAction(
                        myBot.getDrive().actionBuilder(new Pose2d(62, 14 * side, Math.toRadians(-180) * side))
                        // Первый выезд к обелиску
                        .strafeToLinearHeading(new Vector2d(8, -16), Math.toRadians(-147))

                        // Первый забег за шарами
                        .strafeToConstantHeading(new Vector2d(34.5, -16))
                        .strafeToLinearHeading(new Vector2d(34.5, -13), Math.toRadians(-90))
                        .setTangent(Math.toRadians(270))
                        .lineToYConstantHeading(-50)
                        .lineToYConstantHeading(-16)

                        // Второй выезд к обелиску
                        .strafeToLinearHeading(new Vector2d(12, -16), Math.toRadians(-143))
                                .strafeToLinearHeading(new Vector2d(12, -13), Math.toRadians(-90))
                                .setTangent(Math.toRadians(270))
                                .lineToYConstantHeading(-50)
                                .lineToYConstantHeading(-16)
                                .strafeToLinearHeading(new Vector2d(8, -16), Math.toRadians(-147))
                                .strafeToConstantHeading(new Vector2d(-12, -16))
                                .strafeToLinearHeading(new Vector2d(-12, -13), Math.toRadians(-90))
                                .lineToYConstantHeading(-50)
                                .lineToYConstantHeading(-16)
                                .strafeToLinearHeading(new Vector2d(-12, -13), Math.toRadians(-130))


                                // Второй забег за шарами



                        .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
