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
                .waitSeconds(2)
                //First Intake
                .lineToY(60)
                .waitSeconds(1)
                .lineToY(28)
                // Second Shot
                .waitSeconds(2)
                //Second Intake
                .lineToY(20)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(35, 50,Math.toRadians(90)), Math.toRadians(90))
                //Third Shot
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(2)
                //Third Intake
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(60, 45, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(38, 60, Math.toRadians(180)), Math.toRadians(180))
                //Forth Shot
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(90)), Math.toRadians(270))
                .waitSeconds(2)
                //Forth Intake
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(60, 45, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(38, 60, Math.toRadians(180)), Math.toRadians(180))
                //Fifth Shot
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(90)), Math.toRadians(270))
                .waitSeconds(2)
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
