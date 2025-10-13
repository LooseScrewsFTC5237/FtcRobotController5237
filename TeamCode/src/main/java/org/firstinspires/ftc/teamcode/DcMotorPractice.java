package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class DcMotorPractice extends OpMode {
    DecodeRobot bench = new DecodeRobot();

    @Override
    public void init() {
        bench.init(hardwareMap);
    }


    @Override
    public  void loop() {
        double motorSpeed = gamepad1.left_stick_y;

        bench.setMotorSpeed(motorSpeed);

        if (gamepad1.b) {
            bench.setSingleLauncherZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else if (gamepad1.a) {
         bench.setSingleLauncherZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetry.addData("Motor Revs", bench.getMotorRevs());
    }

    public static class MecanumDrivePractice {
        private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
         private IMU imu;

        public void init(HardwareMap hwMap) {
            frontLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");
            backLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
            frontRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
            backRightMotor = hwMap.get(DcMotor.class,"back_right_motor");

            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            imu = hwMap.get(IMU.class, "imu");

            RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

            imu.initialize(new IMU.Parameters(RevOrientation));
        }
        public void drive(double foward, double strafe, double rotate) {
            double frontLeftPower = foward + strafe + rotate;
            double backLeftPower = foward - strafe + rotate;
            double frontRightPower = foward - strafe - rotate;
            double backRightPower = foward + strafe - rotate;

            double maxPower = 1.0;
            double maxSpeed = 1.0;
            //When going to outreach events w/ robot set maxSpeed lower for a slower robot

            maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            frontLeftMotor.setPower(maxSpeed * (frontLeftPower / maxPower));
            frontRightMotor.setPower(maxSpeed * (frontRightPower / maxPower));
            backRightMotor.setPower(maxSpeed * (backRightPower / maxPower));
            backLeftMotor.setPower(maxSpeed * (backLeftPower / maxPower));

        }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
            double theta = Math.atan2(forward, strafe);
            double r = Math.hypot(strafe, forward);

            theta = AngleUnit.normalizeDegrees(theta -
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            double newForward = r * Math.sin(theta);
            double newStrafe = r * Math.cos(theta);

            this.drive(newForward, newStrafe, rotate);
        }
    }
}


