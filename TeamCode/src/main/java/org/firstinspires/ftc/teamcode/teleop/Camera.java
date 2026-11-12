package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.archive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.BeamBrakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraTrackingSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.DepollenaterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemTeleop;

@Config
@TeleOp(name = "Test Camera ", group = "Competition")
public class Camera extends OpMode {
    // region non-magic numbers ★ ▲ ■ ● 👽
    private ShooterSubsystemTeleop shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private FeederSubsystem feederSubsystem;
    private BeamBrakeSubsystem beamBrakeSubsystem;
    private CameraTrackingSubsystems cameraTrackingSubsystems;
    private DepollenaterSubsystem depollenaterSubsystem;
    private IMU imu;


    private double rx;
    private double driveSpeed = 0;
    private double driveSpeedFast = 0.95;
    private double driveSpeedSlow = 0.4;
    private double driveSpeedNormal = 0.8;
    public static double turnSpeed = 0.5;

    public static double autoAimMultuplyer = 0.05;

    private double forward;
    private double strafe;
    private double rotate;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private GoBildaPinpointDriver pinpoint;

    private double headingOffset = 0;
    // endregion

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystemTeleop(hardwareMap, telemetry);
        feederSubsystem = new FeederSubsystem(hardwareMap, telemetry);
        beamBrakeSubsystem = new BeamBrakeSubsystem(hardwareMap, telemetry);
        cameraTrackingSubsystems =new CameraTrackingSubsystems(hardwareMap,telemetry);
        depollenaterSubsystem = new DepollenaterSubsystem(hardwareMap,telemetry);
        imu = hardwareMap.get(IMU.class, "imu");


        beamBrakeSubsystem.Reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        shooterSubsystem.initShooter();
        depollenaterSubsystem.depollenaterUp();

        frontRightDrive = hardwareMap.get(DcMotor.class,"FRDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class,"FLDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class,"BLDrive");
        backRightDrive = hardwareMap.get(DcMotor.class,"BRDrive");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setOffsets(0,0);

        pinpoint.resetPosAndIMU();



    }

    @Override
    public void loop() {

        boolean seeTag = cameraTrackingSubsystems.seeApirltag();

        telemetry.addData("See Apriltag", seeTag);
        telemetry.update();


    }
}