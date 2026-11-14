package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Constants.GARDEN_BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_CAMERA_CHECK_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_CAMERA_CHECK_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_CAMERA_CHECK_VECTOR;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_VECTOR;
import static org.firstinspires.ftc.teamcode.Constants.SLOW_VEL;
import static org.firstinspires.ftc.teamcode.Constants.TO_TOP;
import static org.firstinspires.ftc.teamcode.Constants.TO_LEFT;
import static org.firstinspires.ftc.teamcode.Constants.TO_BOTTOM_RIGHT;
import static org.firstinspires.ftc.teamcode.Constants.TO_BOTTOM_LEFT;
import static org.firstinspires.ftc.teamcode.Constants.PARK_POSE;
import static org.firstinspires.ftc.teamcode.Constants.LOADING_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.Constants.TO_TOP_RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraTrackingSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.DepollenaterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemTeleop;


@Config


@Autonomous
public class GardenSideAuto extends LinearOpMode {

    protected MecanumDrive drive;
    // TODO: create instances of subsystems
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystemTeleop shooterSubsystem;
    private FeederSubsystem feederSubsystem;
    private DepollenaterSubsystem depollenaterSubsystem;
    private CameraTrackingSubsystems cameraTrackingSubsystems;

    private double feederOnTime = 3;




    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        drive = new MecanumDrive(hardwareMap, GARDEN_BEGIN_POSE);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystemTeleop(hardwareMap, telemetry);
        feederSubsystem = new FeederSubsystem(hardwareMap, telemetry);
        cameraTrackingSubsystems = new CameraTrackingSubsystems(hardwareMap, telemetry);
        depollenaterSubsystem = new DepollenaterSubsystem(hardwareMap, telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d beginPose = Constants.GARDEN_BEGIN_POSE;

        // Put depollenater up before starting
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(() -> depollenaterSubsystem.depollenaterUp())
                        .build()
        );

//        PinpointLocalizer pinpoint = (PinpointLocalizer) drive.localizer;
//        GoBildaPinpointDriver.DeviceStatus status;
//        do {
//            pinpoint.update();
//            status = pinpoint.getDeviceStatus();
//            telemetry.addData("Pinpoint Status", status);
//            telemetry.addData("Starting Pose", drive.localizer.getPose());
//            telemetry.update();
//        }  while (status != GoBildaPinpointDriver.DeviceStatus.READY );
//        telemetry.addData("Pinpoint Status", status);
        telemetry.addData("Status", "Ready. Waiting for start.");
        telemetry.addData("Starting Pose", drive.localizer.getPose());
        telemetry.update();

        waitForStart();
        drive.localizer.setPose(GARDEN_BEGIN_POSE);

        // region block 1
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(() -> shooterSubsystem.runShooter() ) // turn on shooter
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feederSubsystem.runIndexerForward()) // shoot
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feederSubsystem.stopIndexer()) // stop shooting
                        .stopAndAdd(() -> intakeSubsystem.runIntakeForward()) //turn on intake
                        .strafeToSplineHeading(INTAKE_GARDEN_VECTOR, INTAKE_GARDEN_HEADING, SLOW_VEL) // intake
                        .waitSeconds(.5)
                        .stopAndAdd(() -> intakeSubsystem.stopIntake()) // turn off intake
                        .strafeToLinearHeading(GARDEN_CAMERA_CHECK_VECTOR, GARDEN_CAMERA_CHECK_HEADING, SLOW_VEL) // camera check
                        .waitSeconds(1)
                        .build()

        );
        // endregion

        // region auto

        if (cameraTrackingSubsystems.seeApirltag()){
            // region block 2
            Actions.runBlocking(
                    drive.actionBuilder(GARDEN_CAMERA_CHECK_POSE)

                            .setTangent(TO_LEFT)
                            .splineToLinearHeading((GARDEN_SHOOT_POSE), TO_LEFT) // move to garden side
                            .stopAndAdd(() -> feederSubsystem.runIndexerForward()) // pew pew
                            .waitSeconds(feederOnTime)
                            .stopAndAdd(() -> feederSubsystem.stopIndexer()) // stop shooter
                            .waitSeconds(1) // camera check

                            .build()

            );
            // endregion

            if (cameraTrackingSubsystems.seeApirltag()){
                // region block 4
                Actions.runBlocking(
                                drive.actionBuilder(GARDEN_BEGIN_POSE)

                                        .setTangent(TO_BOTTOM_RIGHT)
                                        .splineToLinearHeading(PARK_POSE, TO_BOTTOM_RIGHT)

                        .build()


                );

                // endregion
            } else {
                // region block 5
                Actions.runBlocking(
                                drive.actionBuilder(GARDEN_BEGIN_POSE)
                                        .splineToLinearHeading(LOADING_SHOOT_POSE, TO_BOTTOM_LEFT) //move to loading side
                        .stopAndAdd(() -> intakeSubsystem.runIntakeForward())
                        .waitSeconds(3) // intake human player
                        .stopAndAdd(() -> intakeSubsystem.stopIntake()) // stop intaking
                        .stopAndAdd(() -> feederSubsystem.runIndexerForward()) // start shooting
                        .waitSeconds(2) // shoot
                        .stopAndAdd(() -> feederSubsystem.stopIndexer()) // stop shooting
                        .splineToLinearHeading(PARK_POSE, TO_BOTTOM_RIGHT) // park

                                .build()
                );

                // endregion//
            }

        } else {
            // region block 3
            Actions.runBlocking(
                            drive.actionBuilder(GARDEN_CAMERA_CHECK_POSE)
                                    .setTangent(TO_BOTTOM_LEFT)
                                    .splineToLinearHeading(LOADING_SHOOT_POSE, TO_BOTTOM_LEFT) //move to loading side
                                    .stopAndAdd(() -> feederSubsystem.runIndexerForward())  //shoot
                                    .waitSeconds(feederOnTime)
                                    .stopAndAdd(() -> feederSubsystem.stopIndexer()) //stop shooting
                                    .stopAndAdd(() -> intakeSubsystem.runIntakeForward()) //start intaking
                                    .waitSeconds(3)
                                    .stopAndAdd(() -> intakeSubsystem.stopIntake()) //stop intaking
                                    .waitSeconds(1) // camera check

                    .build()
            );
            // endregion

            if (cameraTrackingSubsystems.seeApirltag()){
                // region block 6
                Actions.runBlocking(
                                drive.actionBuilder(LOADING_SHOOT_POSE)
                                        .setTangent(TO_TOP)
                                        .splineToLinearHeading((GARDEN_BEGIN_POSE), TO_LEFT) //move to garden side
                                        .stopAndAdd(() -> feederSubsystem.runIndexerForward()) // shoot
                                        .waitSeconds(feederOnTime)
                                        .stopAndAdd(() -> feederSubsystem.stopIndexer()) // stop shooting
                                        .setTangent(TO_BOTTOM_RIGHT)
                                        .splineToLinearHeading(PARK_POSE, TO_BOTTOM_RIGHT) //park

                        .build()
                );
                // endregion
            } else {
                // region block 7
                Actions.runBlocking(
                                drive.actionBuilder(LOADING_SHOOT_POSE)
                                        .stopAndAdd(() -> feederSubsystem.runIndexerForward())
                                        .waitSeconds(feederOnTime) // shoot
                                        .stopAndAdd(() -> feederSubsystem.stopIndexer())
                                        .setTangent(TO_TOP_RIGHT)
                                        .splineToLinearHeading(PARK_POSE, TO_TOP_RIGHT) //park

                        .build()
                );
                // endregion
            }

        }

        // endregion

    }
}