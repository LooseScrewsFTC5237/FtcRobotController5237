package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingCloseRedNineBallNoDump {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, 37, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(135)), Math.toRadians(0))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-12, 20, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(0)
                .lineToY(42)
                .splineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(135)), Math.toRadians(0))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(12, 20, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(0)
                .lineToY(42)
                .lineToY(20)
                .splineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(135)), Math.toRadians(0))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}
