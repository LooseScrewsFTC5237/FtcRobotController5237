package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.archive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.BeamBrakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraTrackingSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.DepollenaterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemTeleop;


@Config
@TeleOp(name = "BioBuzz No Pods", group = "Competition")
public class BioBuzzNoPodsTeleop extends OpMode {
    // region non magic numbers▲■●★👽
    private ShooterSubsystemTeleop shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private FeederSubsystem feederSubsystem;
    private BeamBrakeSubsystem beamBrakeSubsystem;
    private CameraTrackingSubsystems cameraTrackingSubsystems;
    private DepollenaterSubsystem depollenaterSubsystem;

    private Lights lights;
    private IMU imu;

    private double lockedOn = 1; // TODO change this number
    private double rx;
    private double driveSpeed = 0;
    private double driveSpeedFast = 0.95;
    private double driveSpeedSlow = 0.4;
    private double driveSpeedNormal = 0.8;
    public static double turnSpeed = 0.5;

    public static double autoAimMultuplyer = 0.1;

    public static double autoAimThreshold = 3;

    public static double autoAimK = .2;

    private double forward;
    private double strafe;
    private double rotate;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;


    private double headingOffset = 0;

    private Alliance alliance = Alliance.RED;
    // endregion


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lights = new Lights(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        cameraTrackingSubsystems =new CameraTrackingSubsystems(hardwareMap,telemetry);
        shooterSubsystem = new ShooterSubsystemTeleop(hardwareMap, telemetry, cameraTrackingSubsystems);
        feederSubsystem = new FeederSubsystem(hardwareMap, telemetry);
        beamBrakeSubsystem = new BeamBrakeSubsystem(hardwareMap, telemetry);
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




        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));




}

    @Override
    public void loop() {

        // region Beam break code
        beamBrakeSubsystem.Status();
        // endregion

        // region Shooter code
        shooterSubsystem.pidfTuning();
        shooterSubsystem.runShooter();
        // endregion

        //region light code
        double bearing = cameraTrackingSubsystems.autoAim(alliance);
          if (Math.abs(bearing) < autoAimThreshold && (Math.abs(bearing) > 0.01)) {
            lights.turnRightLockedOnLightOn(0.416);
            lights.turnLeftLockedOnLightOn(0.416);
          } else if (Math.abs(bearing) > autoAimThreshold){
            lights.turnRightLockedOnLightOn(0.28);
            lights.turnLeftLockedOnLightOn(0.28);
          } else  {
            lights.turnRightLockedOnLightOn(0);
            lights.turnLeftLockedOnLightOn(0);


          }
        lights.fadeLight();
        // endregion

        // region Drive code

        if (gamepad1.y){
            imu.resetYaw();
        }

        if (gamepad1.left_bumper) {
            driveSpeed = driveSpeedSlow;
        }else if (gamepad1.right_bumper) {
            driveSpeed = driveSpeedFast;
        }else {
            driveSpeed = driveSpeedNormal;
        }


        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;

        if (gamepad1.right_trigger > 0 && cameraTrackingSubsystems.seeApirltag(alliance) && Math.abs(bearing) > autoAimThreshold){
            rx = (-bearing * autoAimMultuplyer) + autoAimK * bearing / Math.abs(bearing);

        } else {
            rx = gamepad1.right_stick_x;
        }
        telemetry.addData("Camera P", -bearing);

        double rawHeading = AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double robotHeading = rawHeading - headingOffset;

        double rotX = x * Math.cos(robotHeading) - y * Math.sin(robotHeading);
        double rotY = x * Math.sin(-robotHeading) - y * Math.cos(-robotHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower * driveSpeed);
        frontRightDrive.setPower(frontRightPower * driveSpeed);
        backRightDrive.setPower(backRightPower * driveSpeed);
        backLeftDrive.setPower(backLeftPower * driveSpeed);


        // endregion

        // region Intake code
        if ((gamepad2.right_trigger > 0 || gamepad2.right_bumper) & !beamBrakeSubsystem.Status()) {
            intakeSubsystem.runIntakeForward();
        } else if (gamepad2.left_trigger > 0) {
            intakeSubsystem.runIntakeReverse();
            beamBrakeSubsystem.Reset();
        } else {
            intakeSubsystem.stopIntake();
        }
        // endregion

        // region Indexer code
        if(gamepad2.left_trigger > 0){
            feederSubsystem.runIndexerReverse();
            beamBrakeSubsystem.Reset();
        } else if (gamepad2.right_bumper){
            feederSubsystem.runIndexerForward();
            beamBrakeSubsystem.Reset();
        } else {
            feederSubsystem.stopIndexer();
        }

        // endregion

        // region de-pollonater code
        if (gamepad2.y){
            depollenaterSubsystem.DepollenaterDown();
        } else {
            depollenaterSubsystem.depollenaterUp();
        }
        // endregion

        // region switch Alliance; default is "RED"
        if (gamepad1.xWasPressed()){
            alliance = Alliance.BLUE;
        }
        if (gamepad1.bWasPressed()){
            alliance = Alliance.RED;
        }
        // endregion
        telemetry.addData("alliance", alliance);
    }
}