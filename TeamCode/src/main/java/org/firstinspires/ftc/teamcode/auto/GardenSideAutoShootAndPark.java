package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Constants.GARDEN_BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_FIRST_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_FLOWER_APPROACH_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_FLOWER_POSE;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_APPROACH_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_APPROACH_VECTOR;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_VECTOR;
import static org.firstinspires.ftc.teamcode.Constants.LOADING_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.Constants.PARK_POSE;
import static org.firstinspires.ftc.teamcode.Constants.SLOW_SLOW_VEL;
import static org.firstinspires.ftc.teamcode.Constants.SLOW_VEL;
import static org.firstinspires.ftc.teamcode.Constants.TO_BOTTOM;
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

@Config
@Autonomous
public class GardenSideAutoShootAndPark extends LinearOpMode {

    protected MecanumDrive drive;
    // TODO: create instances of subsystems
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystemAuto shooterSubsystem;
    private FeederSubsystem feederSubsystem;
    private DepollenaterSubsystem depollenaterSubsystem;
    private CameraTrackingSubsystems cameraTrackingSubsystems;

    private double feederOnTime = 4;

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
                    .stopAndAdd(() -> shooterSubsystem.runShooter()) // turn on shooter
                    .waitSeconds(2)
                    .splineToLinearHeading(GARDEN_FIRST_SHOOT_POSE, TO_TOP)
                    .stopAndAdd(() -> intakeSubsystem.runIntakeForward())
                    .stopAndAdd(() -> feederSubsystem.runIndexerForward()) // shoot
                    .waitSeconds(feederOnTime)
                    .stopAndAdd(() -> intakeSubsystem.stopIntake())
                    .stopAndAdd(() -> feederSubsystem.stopIndexer()) // stop shooting
                    .stopAndAdd(() -> intakeSubsystem.runIntakeLeft()) // turn on intake
                    .strafeToLinearHeading(INTAKE_GARDEN_APPROACH_VECTOR, INTAKE_GARDEN_APPROACH_HEADING, SLOW_VEL)
                    .strafeToSplineHeading(INTAKE_GARDEN_VECTOR, INTAKE_GARDEN_HEADING, SLOW_SLOW_VEL) // intake
                    .waitSeconds(0.5)
                    .stopAndAdd(() -> intakeSubsystem.stopIntake()) // turn off intake
                    // endregion


                    // region block park
                    .setTangent(TO_BOTTOM_LEFT)
                    .splineToLinearHeading(PARK_POSE, TO_BOTTOM) //park
                    // endregion
        .build()
        );
    }
}
