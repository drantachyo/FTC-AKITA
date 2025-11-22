package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BBCCmeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        boolean isBlue = false;
        double side = isBlue ? -1 : 1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 55, Math.toRadians(360), Math.toRadians(360), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(
                                new Pose2d(61, -14 * side, Math.toRadians(-180) * side))

                        // === 1. Первый выезд к обелиску ===
                        .strafeToLinearHeading(new Vector2d(-13, -16 * side), Math.toRadians(-135) * side)

                        // === 2. Первый забег за шарами ===
                        .strafeToConstantHeading(new Vector2d(34.5, -16 * side))
                        .strafeToLinearHeading(new Vector2d(34.5, -13 * side), Math.toRadians(-90) * side)
                        .setTangent(Math.toRadians(270) * side)
                        .lineToYConstantHeading(-50 * side)
                        .lineToYConstantHeading(-16 * side)

                        // === 3. Второй выезд к обелиску ===
                        .strafeToLinearHeading(new Vector2d(-13, -16 * side), Math.toRadians(-135) * side)

                        // === 4. Второй забег ===
                        .strafeToLinearHeading(new Vector2d(11.6, -13 * side), Math.toRadians(-90) * side)
                        .setTangent(Math.toRadians(270) * side)
                        .lineToYConstantHeading(-50 * side)
                        .lineToYConstantHeading(-16 * side)

                        // === 5. Третий выезд ===
                        .strafeToLinearHeading(new Vector2d(-13, -16 * side), Math.toRadians(-135) * side)

                        // === 6. Третий забег ===
                        .strafeToConstantHeading(new Vector2d(-12, -16 * side))
                        .strafeToLinearHeading(new Vector2d(-12, -13 * side), Math.toRadians(-90) * side)
                        .lineToYConstantHeading(-50 * side)
                        .lineToYConstantHeading(-16 * side)

                        // === 7. Четвёртый выезд к обелиску ===
                        .strafeToLinearHeading(new Vector2d(-12, -13 * side), Math.toRadians(-135) * side)

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
