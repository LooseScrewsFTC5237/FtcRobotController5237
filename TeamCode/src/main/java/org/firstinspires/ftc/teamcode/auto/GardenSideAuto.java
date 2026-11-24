package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Constants.GARDEN_BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_CAMERA_CHECK_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_CAMERA_CHECK_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_CAMERA_CHECK_VECTOR;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_FIRST_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_FLOWER_APPROACH_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_FLOWER_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_APPROACH_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_APPROACH_VECTOR;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_APPROACH_VECTOR_2;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_VECTOR;
import static org.firstinspires.ftc.teamcode.Constants.LOADING_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.Constants.PARK_POSE;
import static org.firstinspires.ftc.teamcode.Constants.SLOW_SLOW_VEL;
import static org.firstinspires.ftc.teamcode.Constants.SLOW_VEL;
import static org.firstinspires.ftc.teamcode.Constants.TO_BOTTOM;
import static org.firstinspires.ftc.teamcode.Constants.TO_BOTTOM_BOTTOM_LEFT;
import static org.firstinspires.ftc.teamcode.Constants.TO_BOTTOM_LEFT;
import static org.firstinspires.ftc.teamcode.Constants.TO_BOTTOM_RIGHT;
import static org.firstinspires.ftc.teamcode.Constants.TO_LEFT;
import static org.firstinspires.ftc.teamcode.Constants.TO_RIGHT;
import static org.firstinspires.ftc.teamcode.Constants.TO_TOP;
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
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemAuto;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemTeleop;

@Config
@Autonomous
public class GardenSideAuto extends LinearOpMode {

    protected MecanumDrive drive;
    // TODO: create instances of subsystems
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystemAuto shooterSubsystem;
    private FeederSubsystem feederSubsystem;
    private DepollenaterSubsystem depollenaterSubsystem;
    private CameraTrackingSubsystems cameraTrackingSubsystems;

    private double feederOnTime = 3.5;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        drive = new MecanumDrive(hardwareMap, GARDEN_BEGIN_POSE);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystemAuto(hardwareMap, telemetry);
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

        telemetry.addData("Status", "Ready. Waiting for start.");
        telemetry.addData("Starting Pose", drive.localizer.getPose());
        telemetry.update();

        waitForStart();

        drive.localizer.setPose(GARDEN_BEGIN_POSE);

        Actions.runBlocking(
            drive.actionBuilder(beginPose)
                    //region block 1
                    .waitSeconds(1)
                    .splineToLinearHeading(GARDEN_FIRST_SHOOT_POSE, TO_BOTTOM)
                    .stopAndAdd(() -> shooterSubsystem.runShooter()) // turn on shooter
                    .waitSeconds(1)
                    .stopAndAdd(() -> intakeSubsystem.runIntakeForward())
                    .stopAndAdd(() -> feederSubsystem.runIndexerForward()) // shoot
                    .waitSeconds(feederOnTime)
                    .stopAndAdd(() -> intakeSubsystem.stopIntake())
                    .stopAndAdd(() -> feederSubsystem.stopIndexer()) // stop shooting
                    .stopAndAdd(() -> intakeSubsystem.runIntakeLeft()) // turn on intake
                    .strafeToSplineHeading(INTAKE_GARDEN_APPROACH_VECTOR_2, INTAKE_GARDEN_HEADING) // intake
                    .strafeToSplineHeading(INTAKE_GARDEN_VECTOR, INTAKE_GARDEN_HEADING, SLOW_VEL) // intake
                    .waitSeconds(0.5)
                    .stopAndAdd(() -> intakeSubsystem.stopIntake()) // turn off intake
                    // endregion

                    // region block 2
                    .setTangent(TO_BOTTOM_LEFT)
                    .stopAndAdd(() -> shooterSubsystem.changeShooterSpeedTo(1240))
                    .splineToLinearHeading(LOADING_SHOOT_POSE, TO_BOTTOM_BOTTOM_LEFT) //move to loading side
                    .stopAndAdd(() -> intakeSubsystem.runIntakeForward())
                    .stopAndAdd(() -> feederSubsystem.runIndexerForward()) // pew pew
                    .waitSeconds(feederOnTime)
                    .stopAndAdd(() -> intakeSubsystem.stopIntake())
                    .stopAndAdd(() -> feederSubsystem.stopIndexer()) // stop shooter
                    // endregion

                    // region block 3
                    .setTangent(TO_TOP_RIGHT)
                    .splineToLinearHeading(GARDEN_FLOWER_APPROACH_POSE, TO_TOP)
                    .stopAndAdd(() -> depollenaterSubsystem.DepollenaterDown())
                    .stopAndAdd(() -> intakeSubsystem.runIntakeForward())
                    .setTangent(TO_RIGHT)
                    .splineToLinearHeading(GARDEN_FLOWER_POSE, TO_TOP_RIGHT, SLOW_VEL)
                    // endregion

                    // region block 4
                    .setTangent(TO_BOTTOM_LEFT)
                    .splineToLinearHeading(LOADING_SHOOT_POSE, TO_BOTTOM_BOTTOM_LEFT ) //move to loading side
                    .stopAndAdd(() -> depollenaterSubsystem.depollenaterUp())
                    .stopAndAdd(() -> feederSubsystem.runIndexerForward()) // shoot
                    .waitSeconds(feederOnTime)
                    .stopAndAdd(() -> intakeSubsystem.stopIntake())
                    .stopAndAdd(() -> feederSubsystem.stopIndexer()) // stop shooting
                    .stopAndAdd(() -> shooterSubsystem.resetShooterSpeed())
                    // endregion

                    // region block park
                    .setTangent(TO_TOP)
                    .splineToLinearHeading(PARK_POSE, TO_RIGHT) //park
                    // endregion
        .build()
        );
    }
}
