package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepFarRedBall {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 28, Math.toRadians(90)))
                //First Shot
                .waitSeconds(1)
                //First Intake
                .lineToY(60)
                .waitSeconds(.5)
                .lineToY(28)
                // Second Shot
                .lineToY(28)
                .waitSeconds(1)
                //Second Intake
                //I stopped here
                .setTangent(Math.toRadians(225))
                .splineToSplineHeading(new Pose2d(52,20,Math.toRadians(90)),Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(35, 50,Math.toRadians(90)), Math.toRadians(90))
                //Third Shot
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(1)
                //Third Intake
                .lineToY(60)
                .waitSeconds(0.5)
                //Forth Shot
                .lineToY(20)
                .waitSeconds(1)
                //Forth Intake
                .lineToY(60)
                .waitSeconds(0.5)
                //Fifth Shot
                .lineToY(20)
                .waitSeconds(1)
                //fifth Intake
                .lineToY(60)
                .waitSeconds(0.5)
                //sixth shot
                .lineToY(20)
                .waitSeconds(1)
                //sixth Intake
                .lineToY(60)
                .waitSeconds(0.5)
                //seventh shot
                .lineToY(20)
                .waitSeconds(1)
                //seventh Intake
                .lineToY(60)
                .waitSeconds(0.5)
                //Park
                .lineToY(40)
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}
