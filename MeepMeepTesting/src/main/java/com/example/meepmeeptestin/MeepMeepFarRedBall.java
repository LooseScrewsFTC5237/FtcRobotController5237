package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
                .strafeToLinearHeading(new Vector2d(56, 20), Math.toRadians(154.5))
                //First Intake
                .setTangent(90)
                .lineToY(60)
                .waitSeconds(.5)
                .lineToY(28)
                // Second Shot
                .splineToLinearHeading(new Pose2d(56, 20, Math.toRadians(158.5)), Math.toRadians(270))
                .lineToY(28)
                .waitSeconds(1)
                //Second Intake
                .setTangent(90)
                .setTangent(Math.toRadians(225))
                .splineToSplineHeading(new Pose2d(52,20,Math.toRadians(90)),Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(35, 50,Math.toRadians(90)), Math.toRadians(90))
                //Third Shot
                .strafeToLinearHeading(new Vector2d(56, 17), Math.toRadians(158.5))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(60, 20, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(1)
                //Third Intake
                .setTangent(90)
                .lineToY(60)
                .waitSeconds(0.5)
                //Forth Shot
                .strafeToLinearHeading(new Vector2d(56, 17), Math.toRadians(153))
                .lineToY(20)
                .waitSeconds(1)
                //Forth Intake
                .setTangent(90)
                .lineToY(60)
                .waitSeconds(0.5)
                //Fifth Shot
                .strafeToLinearHeading(new Vector2d(56, 17), Math.toRadians(153))
                .lineToY(20)
                .waitSeconds(1)
                //Fifth Intake
                .setTangent(90)
                .lineToY(60)
                .waitSeconds(0.5)
//                //Sixth shot
//                .strafeToLinearHeading(new Vector2d(56, 17), Math.toRadians(153))
//                .lineToY(20)
//                .waitSeconds(1)
//                //Sixth Intake
//                .setTangent(90)
//                .lineToY(60)
//                .waitSeconds(0.5)
//                //Seventh shot
//                .strafeToLinearHeading(new Vector2d(56, 17), Math.toRadians(153))
//                .lineToY(20)
//                .waitSeconds(1)
//                //Seventh Intake
//                .setTangent(90)
//                .lineToY(60)
//                .waitSeconds(0.5)
                //Park
                .setTangent(90)
                .lineToY(40)
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}
