package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BBFCmeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 55, Math.toRadians(360), Math.toRadians(360), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(-50, -50, Math.toRadians(234)))
                        // Первый подъезд к обелиску
                        .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(230))

                        // Сбор первых шаров
                        .strafeToLinearHeading(new Vector2d(-11.5, -16), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-11.5, -50), Math.toRadians(270))

                        // Возврат к обелиску для второго выстрела
                        .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(230))

                        // Сбор вторых шаров
                        .strafeToLinearHeading(new Vector2d(11.5, -16), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(11.5, -50), Math.toRadians(270))
                        //возврат к обелиску для третьего выстрела
                        .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(230))

                        // Финальная парковка
                        .strafeToConstantHeading(new Vector2d(14, -14))
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
