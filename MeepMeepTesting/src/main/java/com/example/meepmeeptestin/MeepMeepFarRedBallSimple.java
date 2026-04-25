package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepFarRedBallSimple {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 28, Math.toRadians(180)))
                //First Shot
                .strafeToLinearHeading(new Vector2d(56, 20), Math.toRadians(154.5))
                .waitSeconds(2)

                //First Intake
                .strafeToLinearHeading(new Vector2d(48, 63), Math.toRadians(5))
                .strafeToLinearHeading(new Vector2d(60,64), Math.toRadians(5))
                .waitSeconds(0.6)

                //Park
                .strafeToLinearHeading(new Vector2d(60, 40), Math.toRadians(90))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}
