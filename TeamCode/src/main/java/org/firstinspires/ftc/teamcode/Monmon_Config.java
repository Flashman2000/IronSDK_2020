package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Monmon_Config {

    DcMotor RF = null;
    DcMotor RB = null;
    DcMotor LF = null;
    DcMotor LB = null;

    DcMotor lColl = null;
    DcMotor rColl = null;
    DcMotor spool = null;

    Servo backL = null;
    Servo backR = null;
    Servo leftArm = null;
    Servo rightArm = null;
    Servo grabber = null;
    Servo turner = null;

    Servo frontYk = null;
    Servo backYk = null;

    BNO055IMU imu;

    Orientation angles;

    IronCVDetectorClass detector = new IronCVDetectorClass(-8f/8f, -3f/8f);

    String pos;

    int[] vals;

}
