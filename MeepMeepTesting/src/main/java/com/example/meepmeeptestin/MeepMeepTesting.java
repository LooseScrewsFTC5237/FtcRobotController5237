package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, 30, 0))
                .lineToX(-30)
                .turn(Math.toRadians(-225))
                .lineToY(15)
                .turn(Math.toRadians(180))
                .lineToX(-30)
                .lineToY(15)
                .turn(Math.toRadians(-260))
                .lineToX(0)
                .lineToY(35)
                .turn(Math.toRadians(35))
                .lineToX(0)
                .lineToY(36)

                .build());
//needs to go to 0,50 back and forth
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}