package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//  DcMotorEx shooter;
// DcMotorEx shooter2;
@Autonomous()
public class AutoTest extends LinearOpMode {

    protected MecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {

        // shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        // shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Action simpleAutoPath = drive.actionBuilder(startPose)
                .lineToX(48)
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(48, 12))
                .lineToX(5)
                .build();
        waitForStart();

        if(isStopRequested()) return;
/*
        Actions.runBlocking(simpleAutoPath);
        Pose2d finalPose = drive.pose;


        Action parkingPath = drive.actionBuilder(finalPose)
                .lineToX(0)
                .build();

        Actions.runBlocking(parkingPath);
 */
    }
}

