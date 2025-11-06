package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RBCCmeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        boolean isRed = true;
        double side = isRed ? -1 : 1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 55, Math.toRadians(360), Math.toRadians(360), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(61, -14 * side, Math.toRadians(-180) * side))

                        // --- Первый подъезд к обелиску ---
                        .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(-135) * side)
                        // B-пульс

                        // --- Забег к первым шарам ---
                        .strafeToLinearHeading(new Vector2d(34.5, -15 * side), Math.toRadians(-90) * side)
                        .setTangent(Math.toRadians(270) * side)
                        .lineToYConstantHeading(-55 * side)
                        .lineToYConstantHeading(-16 * side)
                        // Интейк ON

                        // --- Возврат к обелиску для второго выстрела ---
                        .strafeToLinearHeading(new Vector2d(-19, -16 * side), Math.toRadians(-135) * side)
                        // B-пульс

                        // --- Выезд ко вторым шарам ---
                        .lineToXLinearHeading(30, Math.toRadians(-90) * side)
                        .splineToSplineHeading(new Pose2d(44, -60 * side, Math.toRadians(0) * side), Math.toRadians(-90) * side)
                        .strafeToLinearHeading(new Vector2d(44, -60 * side), Math.toRadians(0) * side)
                        .setTangent(Math.toRadians(0))
                        .lineToXConstantHeading(60)
                        // Интейк ON

                        // --- Финальный подъезд к обелиску ---
                        .strafeToLinearHeading(new Vector2d(-13, -16 * side), Math.toRadians(-135) * side)
                        // B-пульс
                        .strafeToLinearHeading(new Vector2d(44, -16 * side), Math.toRadians(-180))

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
