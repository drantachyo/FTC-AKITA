package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BBCCmeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 55, Math.toRadians(360), Math.toRadians(360), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(
                                new Pose2d(61, -14, Math.toRadians(-180)))

                        // === 1. Первый выезд к обелиску ===
                        .strafeToLinearHeading(new Vector2d(-12, -16), Math.toRadians(-135))
                        // === 2. Первый забег за шарами ===
                        .turn(Math.toRadians(45))
                        .strafeToConstantHeading(new Vector2d(-12, -53))
                        .strafeToConstantHeading(new Vector2d(-3, -40))
                        .strafeToConstantHeading(new Vector2d(-3, -53))

                        // === 3. Второй выезд к обелиску ===
                        .strafeToLinearHeading(new Vector2d(-12, -16), Math.toRadians(-135))

                        // === 4. Второй забег ===
                        .strafeToLinearHeading(new Vector2d(11.5, -16), Math.toRadians(-90))
                        .setTangent(Math.toRadians(270))
                        .lineToYConstantHeading(-60)
                        .lineToYConstantHeading(-50)


                        // === 5. Третий выезд ===
                        .strafeToLinearHeading(new Vector2d(-12, -16), Math.toRadians(-135))

                        // === 6. Третий забег ===
                        .strafeToLinearHeading(new Vector2d(34.5, -14), Math.toRadians(-90))
                        .setTangent(Math.toRadians(-90))
                        .lineToYConstantHeading(-60)


                        // === 7. Четвёртый выезд к обелиску ===
                        .strafeToLinearHeading(new Vector2d(50, -14), Math.toRadians(-150))

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}
