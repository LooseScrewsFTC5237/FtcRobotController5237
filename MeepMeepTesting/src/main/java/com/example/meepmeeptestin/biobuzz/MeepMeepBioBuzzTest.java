package com.example.meepmeeptestin.biobuzz;

import static com.example.meepmeeptestin.Constants.GARDEN_BEGIN_POSE;
import static com.example.meepmeeptestin.Constants.GARDEN_CAMERA_CHECK_HEADING;
import static com.example.meepmeeptestin.Constants.GARDEN_CAMERA_CHECK_POSE;
import static com.example.meepmeeptestin.Constants.GARDEN_CAMERA_CHECK_VECTOR;
import static com.example.meepmeeptestin.Constants.GARDEN_SHOOT_POSE;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_APPROACH_HEADING;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_APPROACH_VECTOR;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_HEADING;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_VECTOR;
import static com.example.meepmeeptestin.Constants.SLOW_SLOW_VEL;
import static com.example.meepmeeptestin.Constants.SLOW_VEL;
import static com.example.meepmeeptestin.Constants.TO_BOTTOM;
import static com.example.meepmeeptestin.Constants.TO_TOP;
import static com.example.meepmeeptestin.Constants.TO_LEFT;
import static com.example.meepmeeptestin.Constants.TO_BOTTOM_RIGHT;
import static com.example.meepmeeptestin.Constants.PARK_POSE;
import static com.example.meepmeeptestin.Constants.LOADING_SHOOT_POSE;
import static com.example.meepmeeptestin.Constants.INTAKE_LOADING_POSE;
import static com.example.meepmeeptestin.Constants.SLOW_VEL;
import static com.example.meepmeeptestin.Constants.TO_TOP_RIGHT;
import static com.example.meepmeeptestin.Constants.TO_BOTTOM_LEFT;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepBioBuzzTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(GARDEN_BEGIN_POSE)

                // region block 1
                .waitSeconds(2) // Shoot
                .strafeToLinearHeading(INTAKE_GARDEN_APPROACH_VECTOR, INTAKE_GARDEN_APPROACH_HEADING, SLOW_VEL)
                .strafeToSplineHeading(INTAKE_GARDEN_VECTOR, INTAKE_GARDEN_HEADING, SLOW_SLOW_VEL) // Intake
                .waitSeconds(0.5)
                .setTangent(TO_BOTTOM_LEFT)
                .strafeToLinearHeading(GARDEN_CAMERA_CHECK_VECTOR, GARDEN_CAMERA_CHECK_HEADING)
                .waitSeconds(1)
                // endregion

                //  region block 2 (off)
//                .setTangent(180)
//                .splineToLinearHeading(GARDEN_SHOOT_POSE, Math.toRadians(180))
//                .waitSeconds(2) //shoot
//                .waitSeconds(1) //camera check
                //  endregion

                // region block 3

                .setTangent(TO_BOTTOM_LEFT)
                .splineToLinearHeading(LOADING_SHOOT_POSE, TO_BOTTOM) //move to loading side
                .waitSeconds(2) // shoot
                .waitSeconds(1) // camera check

                // endregion

                // region block 4 (off)
//                .setTangent(Math.toRadians(330))
//                .splineToLinearHeading(PARK_POSE, TO_BOTTOM_RIGHT)
                // endregion

                // region block 5 (off)
//                .splineToLinearHeading(LOADING_SHOOT_POSE, TO_BOTTOM_LEFT) //move to loading side
//                .waitSeconds(3) // intake human player
//                .waitSeconds(2) // shoot
//                .splineToLinearHeading(new Pose2d(PARK_POSE), TO_BOTTOM_RIGHT) // park
                // endregion

                // region block 6 (off)

//                .setTangent(TO_TOP)
//                .splineToLinearHeading(GARDEN_SHOOT_POSE), TO_TOP_LEFT) //move to garden side
//                .waitSeconds(2)// shoot
//                .setTangent(TO_BOTTOM_RIGHT)
//                .splineToLinearHeading(PARK_POSE, TO_BOTTOM_RIGHT) // park

                // endregion
                // region block 6.1
                .waitSeconds(1) // shoot
                .setTangent(TO_TOP_RIGHT)
                .splineToLinearHeading(INTAKE_LOADING_POSE, TO_TOP_RIGHT, SLOW_VEL) // intake
                .waitSeconds(1) // intake

                .splineToLinearHeading(LOADING_SHOOT_POSE, TO_BOTTOM_LEFT) // shoot

                // endregion
                // region block 7
                .waitSeconds(2) // shoot
                .setTangent(TO_TOP_RIGHT)
                .splineToLinearHeading(PARK_POSE, TO_TOP_RIGHT) //park
                // endregion

                .build());

        Image img = null;
        try {
            // Provide the absolute path to your image file
            img = ImageIO.read(new File("MeepMeepTesting\\biobuzz.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}