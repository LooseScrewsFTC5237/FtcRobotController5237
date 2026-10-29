package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;


@Config


@Autonomous
public class BlueScoring extends LinearOpMode {

    protected MecanumDrive drive;
    // TODO: create instances of subsystems
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private FeederSubsystem feederSubsystem;
    PoseMap poseMap;
    private double feederOnTime = 2;


    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(14, 60.5, Math.toRadians(270));


        Actions.runBlocking(
                new RaceAction(
                        drive.actionBuilder(beginPose, poseMap)
                                .stopAndAdd(() -> shooterSubsystem.runShooter() )
                                .stopAndAdd(() -> feederSubsystem.runIndexerForward())
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feederSubsystem.stopIndexer())
                                .strafeToSplineHeading(new Vector2d(60.5,60.5), Math.toRadians(270))

                                .build()
                )


        );

        Actions.runBlocking(
                new RaceAction(
                        drive.actionBuilder(beginPose, poseMap)
                                .stopAndAdd(() -> shooterSubsystem.runShooter() )
                                .stopAndAdd(() -> feederSubsystem.runIndexerForward())
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feederSubsystem.stopIndexer())


                                .build()
                )


        );

        Actions.runBlocking(
                new RaceAction(
                        drive.actionBuilder(beginPose, poseMap)
                                .stopAndAdd(() -> shooterSubsystem.runShooter() )
                                .stopAndAdd(() -> feederSubsystem.runIndexerForward())
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feederSubsystem.stopIndexer())


                                .build()
                )


        );

        Actions.runBlocking(
                new RaceAction(
                        drive.actionBuilder(beginPose, poseMap)
                                .stopAndAdd(() -> shooterSubsystem.runShooter() )
                                .stopAndAdd(() -> feederSubsystem.runIndexerForward())
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feederSubsystem.stopIndexer())


                                .build()
                )


        );

        Actions.runBlocking(
                new RaceAction(
                        drive.actionBuilder(beginPose, poseMap)
                                .stopAndAdd(() -> shooterSubsystem.runShooter() )
                                .stopAndAdd(() -> feederSubsystem.runIndexerForward())
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feederSubsystem.stopIndexer())


                                .build()
                )


        );

        Actions.runBlocking(
                new RaceAction(
                        drive.actionBuilder(beginPose, poseMap)
                                .stopAndAdd(() -> shooterSubsystem.runShooter() )
                                .stopAndAdd(() -> feederSubsystem.runIndexerForward())
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feederSubsystem.stopIndexer())


                                .build()
                )


        );

        Actions.runBlocking(
                new RaceAction(
                        drive.actionBuilder(beginPose, poseMap)
                                .stopAndAdd(() -> shooterSubsystem.runShooter() )
                                .stopAndAdd(() -> feederSubsystem.runIndexerForward())
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feederSubsystem.stopIndexer())


                                .build()
                )


        );
    }
}
