package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

public class Constants {

    public static final int EXAMPLE = 1;
    public static final Pose2d GARDEN_BEGIN_POSE = new Pose2d(14,60.5,Math.toRadians(180));
    public static final Pose2d GARDEN_SHOOT_POSE = new Pose2d(14,60.5,Math.toRadians(180));
    public static final Pose2d GARDEN_FIRST_SHOOT_POSE = new Pose2d(14,58.5,Math.toRadians(180));


    public static final Pose2d LOADING_SHOOT_POSE = new Pose2d(40,-45,Math.toRadians(40));

    public static final double TO_TOP = Math.toRadians(90);
    public static final double TO_BOTTOM = Math.toRadians(270);
    public static final double TO_LEFT = Math.toRadians(180);
    public static final double TO_RIGHT = Math.toRadians(0);

    public static final double TO_BOTTOM_LEFT = Math.toRadians(225);
    public static final double TO_BOTTOM_RIGHT = Math.toRadians(315);
    public static final double TO_TOP_LEFT = Math.toRadians(135);
    public static final double TO_TOP_RIGHT = Math.toRadians(45);

    public static final Vector2d INTAKE_GARDEN_VECTOR = new Vector2d(61,60.5);
    public static final double INTAKE_GARDEN_HEADING = Math.toRadians(180);


    public static final Pose2d INTAKE_LOADING_POSE = new Pose2d(60.0, -34, Math.toRadians(0));

    public static final Pose2d GARDEN_FLOWER_POSE = new Pose2d(60,23, Math.toRadians(0));

    public static final Pose2d GARDEN_FLOWER_APPROACH_POSE = new Pose2d(40,23, Math.toRadians(0));

    public static final Vector2d INTAKE_GARDEN_APPROACH_VECTOR = new Vector2d(55,57.5);
    public static final double INTAKE_GARDEN_APPROACH_HEADING = Math.toRadians(225);

    public static final TranslationalVelConstraint FAST_VEL = new TranslationalVelConstraint(500);
    public static final TranslationalVelConstraint SLOW_VEL = new TranslationalVelConstraint(25);
    public static final TranslationalVelConstraint SLOW_SLOW_VEL = new TranslationalVelConstraint(7.5);

    public static final Vector2d GARDEN_CAMERA_CHECK_VECTOR = new Vector2d(55,55);
    public static final double GARDEN_CAMERA_CHECK_HEADING = Math.toRadians(130);
    public static final Pose2d GARDEN_CAMERA_CHECK_POSE = new Pose2d(55, 55, Math.toRadians(130));

    public static final Pose2d PARK_POSE = new Pose2d(57, -20, Math.toRadians(90));

    //Loading side variables
    public static final Pose2d LOADING_BEGIN_POSE = new Pose2d(9,-60.5, Math.toRadians(0));
    public static final Pose2d LOADING_FLOWER_APPROACH_POSE = new Pose2d(23,-43, Math.toRadians(270));
    public static final Pose2d LOADING_FLOWER_POSE = new Pose2d(23,-55.8, Math.toRadians(270));
    public static final Pose2d LOADING_SHOOT_POSE_2 = new Pose2d(9,-60.5, Math.toRadians(0));
    public static final Pose2d LOADING_PARK_POSE = new Pose2d(60.5,-45, Math.toRadians(0));


}