package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RBFCmeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        boolean isRed = true;
        double side = isRed ? -1 : 1; // красная база = -1

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 55, Math.toRadians(360), Math.toRadians(360), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(-50, -50 * side, Math.toRadians(234 * side)))

                        // Первый подъезд к обелиску
                        .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(230 * side))

                        // Сбор первых шаров
                        .strafeToLinearHeading(new Vector2d(-11.5, -16 * side), Math.toRadians(270 * side))
                        .strafeToLinearHeading(new Vector2d(-11.5, -50 * side), Math.toRadians(270 * side))

                        // Возврат к обелиску для второго выстрела
                        .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(230 * side))

                        // Сбор вторых шаров
                        .strafeToLinearHeading(new Vector2d(11.5, -16 * side), Math.toRadians(270 * side))
                        .strafeToLinearHeading(new Vector2d(11.5, -50 * side), Math.toRadians(270 * side))

                        // Возврат к обелиску для третьего выстрела
                        .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(230 * side))

                        // Финальная парковка
                        .strafeToConstantHeading(new Vector2d(14, -14 * side))
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
