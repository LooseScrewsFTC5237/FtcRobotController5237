package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TwentyTwentyFiveJava.shooterSpeed;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

@Config
public class FarAuto6Ball extends LinearOpMode {

    boolean isRed = false;
    @Autonomous()
    public static class RedFarAuto6Ball extends FarAuto6Ball {
        public RedFarAuto6Ball() {
            super(new Pose2d(60, 30, Math.toRadians(180)), new IdentityPoseMap());
            isRed = true;
        }
    }

    @Autonomous()
    public static class BlueFarAuto6Ball extends FarAuto6Ball {
        public BlueFarAuto6Ball() {
            super(
                    new Pose2d(60, -30, Math.toRadians(180)),
                    pose -> new Pose2dDual<>(
                            new Vector2dDual<>(
                                    pose.position.x,
                                    pose.position.y.unaryMinus()
                            ),
                            pose.heading.inverse()
                    )
            );
        }
    }

    protected MecanumDrive drive;

    DcMotor intake;
    DcMotor feeder;
    DcMotorEx shooter;
    DcMotorEx shooter2;
    Hood hood = new Hood();
    private Limelight3A limelight;

    public static boolean UPDATE_FLYWHEEL_PID = false;

    public static double Redoffset = 5;
    public static double Blueoffset = -1;
    public static double BEARING_THRESHOLD = 0.25; // Angled towards the tag (degrees)
    public static double TURN_GAIN   =  0.04  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double TURN_STATIC = 0.1;
    public static double MAX_AUTO_TURN  = 0.3;

    public static double feederOnTime = 0.125;
    PoseMap poseMap;
    Pose2d startingPose;

    Action autoAimAction() {
        return new Action() {

            long startTimeMillis = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                long now = System.currentTimeMillis();
                if (startTimeMillis == 0) {
                    startTimeMillis = now;
                }

                // Do your auto aim code here
                double turn = 0;

                LLResult llResult = limelight.getLatestResult();
                double headingError = llResult.getTx();
                double headingOffset = isRed ? Redoffset : Blueoffset;
                double offsetError = headingError + headingOffset;
                if (Math.abs(offsetError) < BEARING_THRESHOLD) {
                    return false;
                }
                turn = Range.clip((offsetError + (Math.signum(offsetError) * TURN_STATIC)) * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turn * -1));

                return (now - startTimeMillis < 2_000);
            }
        };
    }

    Action shooterCheckAction() {
        return new Action() {

            long startTimeMillis = 0;
            double currentShooterVelocity;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                long now = System.currentTimeMillis();
                if (startTimeMillis == 0) {
                    startTimeMillis = now;
                }

                currentShooterVelocity = shooter.getVelocity();
                telemetryPacket.put("Current Velocity", currentShooterVelocity);
                telemetryPacket.put("Target Velocity", shooterSpeed);
                if (Math.abs(currentShooterVelocity - shooterSpeed) < 50  ) {
                    return false;
                }

                return (now - startTimeMillis < 1_000);

            }
        };
    }

    public FarAuto6Ball(Pose2d startingPose, PoseMap poseMap) {
        this.poseMap = poseMap;
        this.startingPose = startingPose;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.get(DcMotor.class, "Intake");
        feeder = hardwareMap.get(DcMotor.class, "Shooter Feeder");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        hood.init(hardwareMap);
        hood.setServoPos(0.42);
        double shooterSpeed = 1110;
        double currentShooterVelocity = shooter.getVelocity();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        PIDFCoefficients c = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!UPDATE_FLYWHEEL_PID) {
            TwentyTwentyFiveJava.FLYWHEEL_P = c.p;
            TwentyTwentyFiveJava.FLYWHEEL_I = c.i;
            TwentyTwentyFiveJava.FLYWHEEL_D = c.d;
            TwentyTwentyFiveJava.FLYWHEEL_F = c.f;
        }
        pidTuner();

        Pose2d beginPose = startingPose;// new Pose2d(60, 30, Math.toRadians(180));
        drive = new MecanumDrive(hardwareMap, beginPose);
        limelight.start();
        telemetry.addData("Current Velocity", 0);
        telemetry.addData("Target Velocity", 0);
        telemetry.addData("Feeder Speed", 0);
        waitForStart();
        pidTuner();

        Actions.runBlocking(
                drive.actionBuilder(beginPose, poseMap)

                        // Turn on motors
                        .stopAndAdd(() -> {
                            intake.setPower(1.0);
                            feeder.setPower(0.0);
                            shooter.setVelocity(shooterSpeed);
                            shooter2.setVelocity(shooterSpeed);
                        })

                        // Prep First Three Shots
                        .strafeToLinearHeading(new Vector2d(56, 23), Math.toRadians(155.5))
                        .stopAndAdd(autoAimAction())

                        // Take First Three Shots
                        .stopAndAdd(shooterCheckAction())
                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (1st time)
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .stopAndAdd(shooterCheckAction())
                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (2nd time)
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .stopAndAdd(shooterCheckAction())
                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (3rd time)
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF

                        // Intake Corner Artifacts
                        .waitSeconds(0.6)
                        .turnTo(Math.toRadians(-280))
                        .strafeToLinearHeading(new Vector2d(60, 34), Math.toRadians(-280))
                        .lineToX(72 /*, new TranslationalVelConstraint(21), null */)
                        .lineToX(67 /*, new TranslationalVelConstraint(21), null */)
                        .stopAndAdd(() -> intake.setPower(0)) // Turn off intake

                        // Prep Second Three Shots
                        .strafeToLinearHeading(new Vector2d(56, 23), Math.toRadians(155.5))
                        .stopAndAdd(autoAimAction())
                        .stopAndAdd(() -> intake.setPower(1)) // Turn intake back on

                        // Take Second Three Shots
                        .stopAndAdd(shooterCheckAction())
                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (1st time)
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .stopAndAdd(shooterCheckAction())
                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (2nd time)
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF
                        .stopAndAdd(shooterCheckAction())
                        .stopAndAdd(() -> feeder.setPower(1.0)) // Feeder ON (3rd time)
                        .waitSeconds(feederOnTime)
                        .stopAndAdd(() -> feeder.setPower(0.0)) // Feeder OFF

                        // Park
                        .strafeToLinearHeading(new Vector2d(56, 36), Math.toRadians(75))

                        .build());

        if(isStopRequested()) return;

        intake.setPower(0);
        feeder.setPower(0);
        shooter.setPower(0);
        shooter2.setPower(0);
    }
    private void pidTuner() {
        if (UPDATE_FLYWHEEL_PID) {
            PIDFCoefficients c = new PIDFCoefficients(TwentyTwentyFiveJava.FLYWHEEL_P, TwentyTwentyFiveJava.FLYWHEEL_I, TwentyTwentyFiveJava.FLYWHEEL_D, TwentyTwentyFiveJava.FLYWHEEL_F);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
            shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
            UPDATE_FLYWHEEL_PID = false;
        }
        telemetry.addData("Flywheel P", TwentyTwentyFiveJava.FLYWHEEL_P);
        telemetry.addData("Flywheel I", TwentyTwentyFiveJava.FLYWHEEL_I);
        telemetry.addData("Flywheel D", TwentyTwentyFiveJava.FLYWHEEL_D);
        telemetry.addData("Flywheel F", TwentyTwentyFiveJava.FLYWHEEL_F);
    }

}
