package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Disabled
public class RobotConfiguration {
    private static final boolean FORWARD = true;
    private static final boolean REVERSE = false;
    private HardwareMap hw;

    public RobotConfiguration(HardwareMap hardwareMap) {
        this.hw = hardwareMap;




    }
    public String getFrontLeftDriveName() {return "FL Drive";}
    public String getFrontRightDriveName() {return "FR Drive";}
    public String getBackLeftDriveName() {return "BL Drive";}
    public String getBackRightDriveName() {return "BR Drive";}
    public String getPinpointName() {return "pinpoint";}
    public String getImuName() {return "imu";}

    public DcMotorEx getFrontLeftDrive() {return getDriveMotor(getFrontLeftDriveName(), REVERSE);}
    public DcMotorEx getFrontRightDrive() {return getDriveMotor(getFrontRightDriveName(), FORWARD);}
    public DcMotorEx getBackLeftDrive() {return getDriveMotor(getBackLeftDriveName(), REVERSE);}
    public DcMotorEx getBackRightDrive() {return getDriveMotor(getBackRightDriveName(), FORWARD);}

    private DcMotorEx getDriveMotor(String name, boolean isForward) {
     DcMotorEx motor = hw.get(DcMotorEx.class, name);
     motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     motor.setDirection( isForward ? DcMotorSimple.Direction.FORWARD: DcMotorSimple.Direction.REVERSE);
     motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     return motor;
    }
}
