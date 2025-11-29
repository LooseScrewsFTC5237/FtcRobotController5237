package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBlueFarAutoFour {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -30, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(56, -20, Math.toRadians(192)), Math.toRadians(0))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(35, -20, Math.toRadians(-91)), Math.toRadians(0))
                .waitSeconds(0)
                .lineToY(-50)
                .splineToLinearHeading(new Pose2d(56, -20, Math.toRadians(192)), Math.toRadians(0))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(40, -60, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0)
                .lineToX(58)
                .waitSeconds(0)
                .splineToLinearHeading(new Pose2d(38, -50, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0)
                .splineToLinearHeading(new Pose2d(56, -20, Math.toRadians(192)), Math.toRadians(0))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(40, -20, Math.toRadians(90)), Math.toRadians(0))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}