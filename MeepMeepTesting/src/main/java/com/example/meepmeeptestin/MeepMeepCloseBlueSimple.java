package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCloseBlueSimple {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d dumpPose1 = new Pose2d(5, -68, Math.toRadians(225));
        double dumpTangent1 = Math.toRadians(270);
        Vector2d shootPose = new Vector2d(-16, -16);
        Pose2d dumpPose2 = new Pose2d(20, -63, Math.toRadians(225));
        double dumpTangent2 = Math.toRadians(0);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, -37, Math.toRadians(0)))

                //First Shot
                .strafeToLinearHeading(new Vector2d(-16, -16), Math.toRadians(217))
                .waitSeconds(2)

                //Park
                .strafeToLinearHeading(new Vector2d(-60, -16), Math.toRadians(270))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}
