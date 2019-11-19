package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.IronCV.IronCVDetectorClass;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Monmon_Config {

    public DcMotor RF = null;
    public DcMotor RB = null;
    public DcMotor LF = null;
    public DcMotor LB = null;

    public DcMotor lColl = null;
    public DcMotor rColl = null;
    public DcMotor spool = null;

    public Servo backL = null;
    public Servo backR = null;
    public Servo leftArm = null;
    public Servo rightArm = null;
    public Servo grabber = null;
    public Servo turner = null;

    public Servo frontYk = null;
    public Servo backYk = null;

    public BNO055IMU imu;

    public Orientation angles;

    public IronCVDetectorClass detector = new IronCVDetectorClass(-8f/8f, -3f/8f);

    public String pos;

    public int[] vals;

}
