package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingRedFarAutoFour {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 30, Math.toRadians(-180)))
                .splineToLinearHeading(new Pose2d(56, 20, Math.toRadians(155.5)), Math.toRadians(0))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(60, 34, Math.toRadians(-270)), Math.toRadians(0))
                .waitSeconds(0)
                .lineToY(60)
                .lineToY(34)
                .splineToLinearHeading(new Pose2d(56, 20, Math.toRadians(155.5)), Math.toRadians(0))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(60, 34, Math.toRadians(-270)), Math.toRadians(0))
                .waitSeconds(0)
                .lineToY(60)
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}