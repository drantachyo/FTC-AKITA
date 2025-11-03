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
                // maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth
                .setConstraints(70, 55, Math.toRadians(450), Math.toRadians(450), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(62, -14, Math.toRadians(180)))
                        // Выплюнуть шары загруженные
                        .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(229))
                        .strafeToLinearHeading(new Vector2d(-19.1, -16), Math.toRadians(229))

                        // Подъехать к шарам 3
                        .strafeToLinearHeading(new Vector2d(35, -30), Math.toRadians(270))
                        .setTangent(Math.toRadians(270))
                        .lineToYConstantHeading(-55)

                        // Выплюнуть шары 3
                        .strafeToLinearHeading(new Vector2d(-19, -16), Math.toRadians(229))
                        .strafeToLinearHeading(new Vector2d(-19.1, -16), Math.toRadians(229))

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
