package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RBCCmeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 55, Math.toRadians(360), Math.toRadians(360), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(62, 14, Math.toRadians(-180)))
                        // первый выстрел
                        .strafeToLinearHeading(new Vector2d(8, 16), Math.toRadians(148))
                        .strafeToLinearHeading(new Vector2d(8.1, 16), Math.toRadians(148))


                        // движение к первым шарам
                        .splineToSplineHeading(new Pose2d(33, 35, Math.toRadians(90)), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(33, 35), Math.toRadians(90)) // выравнивание
                        .setTangent(Math.toRadians(-270))
                        .lineToYConstantHeading(55)
                        .lineToYConstantHeading(16)

                        // возврат для второго выстрела
                        .strafeToLinearHeading(new Vector2d(8, 16), Math.toRadians(148)) // выравнивание


                        // выезд ко вторым шарам
                        .lineToXLinearHeading(30, Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(44, 60, Math.toRadians(0)), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(44, 60), Math.toRadians(0)) // выравнивание
                        .setTangent(Math.toRadians(0))
                        .lineToXConstantHeading(59)
                        .strafeToConstantHeading(new Vector2d(35, 30))

                        // финальный возврат на позицию
                        .strafeToLinearHeading(new Vector2d(8, 16), Math.toRadians(148)) // финальное выравнивание



                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
