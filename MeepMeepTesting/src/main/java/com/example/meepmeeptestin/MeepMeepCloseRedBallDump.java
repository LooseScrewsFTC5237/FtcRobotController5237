package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCloseRedBallDump {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, 37, Math.toRadians(0)))
                //First Shot
                .splineToLinearHeading(new Pose2d(-16, 16, Math.toRadians(135)), Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(0)
                //Intake Middle Line
                .turn(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(12, 50,Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0)
                //Second Shot
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-16, 16,Math.toRadians(135)), Math.toRadians(180))
                .waitSeconds(1)
                //Dump'N Intake
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(10, 55, Math.toRadians(120)), Math.toRadians(90))
                .waitSeconds(1)
                //Third Shot
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-16, 16, Math.toRadians(135)), Math.toRadians(180))
                .waitSeconds(1)
                //Intake Goal Side Line
                .turn(Math.toRadians(-45))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-16, 50, Math.toRadians(90)), Math.toRadians(270))
                //Fourth Shot
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-16, 16, Math.toRadians(135)), Math.toRadians(90))
                .waitSeconds(1)
                //Dump'N Intake2
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(10, 55, Math.toRadians(120)), Math.toRadians(90))
                .waitSeconds(1)
                //Fifth Shot
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-16, 16, Math.toRadians(135)), Math.toRadians(180))
                .waitSeconds(1)
                //Park
                .splineToLinearHeading(new Pose2d(-16, 35, Math.toRadians(90)), Math.toRadians(90))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}
