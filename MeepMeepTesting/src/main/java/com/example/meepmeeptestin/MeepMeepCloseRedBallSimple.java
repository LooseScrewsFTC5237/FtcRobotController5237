package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCloseRedBallSimple {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d dumpPose1 = new Pose2d(10, 57, Math.toRadians(135));
        double dumpTangent1 = Math.toRadians(0);
        Pose2d shootPose = new Pose2d(-16, 16, Math.toRadians(135));
        Pose2d dumpPose2 = new Pose2d(25, 60, Math.toRadians(135));
        double dumpTangent2 = Math.toRadians(0);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, 37, Math.toRadians(0)))

                //First Shot
                .strafeToLinearHeading(new Vector2d (-16, 16) , Math.toRadians(135))
                .waitSeconds(2)

                //Park
                .strafeToLinearHeading(new Vector2d(-60, 16), Math.toRadians(90))

                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}
