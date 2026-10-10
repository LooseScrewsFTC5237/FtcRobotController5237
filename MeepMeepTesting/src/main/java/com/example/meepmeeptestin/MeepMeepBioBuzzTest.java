package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBioBuzzTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(6, 0))

                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
        }