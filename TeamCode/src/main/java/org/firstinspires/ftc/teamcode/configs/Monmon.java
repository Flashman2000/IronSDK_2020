package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;



public class Monmon extends Monmon_Config{

    public Monmon(){}

    public void initAuto(HardwareMap hardwareMap, Telemetry telemetry){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu 1");

        imu.initialize(parameters);

        LF = hardwareMap.get(DcMotor.class, "lf");
        LB = hardwareMap.get(DcMotor.class, "lb");
        RB = hardwareMap.get(DcMotor.class, "rb");
        RF = hardwareMap.get(DcMotor.class, "rf");

        lColl = hardwareMap.get(DcMotor.class, "lc");
        rColl = hardwareMap.get(DcMotor.class, "rc");
        spool = hardwareMap.get(DcMotor.class, "spool");

        backL = hardwareMap.get(Servo.class, "bl");
        backR = hardwareMap.get(Servo.class, "br");
        leftArm = hardwareMap.get(Servo.class, "lcoll");
        rightArm = hardwareMap.get(Servo.class, "rcoll");
        grabber = hardwareMap.get(Servo.class, "grab");
        turner = hardwareMap.get(Servo.class, "turn");

        frontYk = hardwareMap.get(Servo.class, "frntyk");
        backYk = hardwareMap.get(Servo.class, "bckyk");


        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        rColl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lColl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        detector.camSetup(hardwareMap);

        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addLine("Ready");
        telemetry.update();

    }

    public String detectSkystone(){

        detector.updateVals();
        vals = detector.getVals();

        if(vals[1] < 200 && vals[0] > 210 && vals[2] > 210){
            return "Left";
        }else if(vals[1] > 210 && vals[0] < 200 && vals[2] > 210){
            return "Center";
        }else if(vals[1] > 210 && vals[0] > 210 && vals[2] < 200){
            return "Right";
        }else{
            return "NOT FOUND";
        }

    }

    public void killAll(){
        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);
    }

    public void resetEncoders(){
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void fwdWithEncoder(double pwr, int pulses, LinearOpMode opmode){
        while(LF.getCurrentPosition() < pulses && opmode.opModeIsActive()){
            LF.setPower(pwr);
            LB.setPower(pwr);
            RF.setPower(pwr);
            RB.setPower(pwr);
        }
    }

    public void bckWithEncoder(double pwr, int pulses, LinearOpMode opmode){
        while(LF.getCurrentPosition() > -pulses && opmode.opModeIsActive()){
            LF.setPower(-pwr);
            LB.setPower(-pwr);
            RF.setPower(-pwr);
            RB.setPower(-pwr);
        }
    }

    public void strafeLeftWithEnc(double pwr, int pulses, LinearOpMode opmode){
        while(LF.getCurrentPosition() > -pulses && opmode.opModeIsActive()){
            LF.setPower(-pwr);
            LB.setPower(pwr);
            RF.setPower(pwr);
            RB.setPower(-pwr);
        }
    }

    public void strafeRightWithEnc(double pwr, int pulses, LinearOpMode opmode){
        while(LF.getCurrentPosition() < pulses && opmode.opModeIsActive()){
            LF.setPower(pwr);
            LB.setPower(-pwr);
            RF.setPower(-pwr);
            RB.setPower(pwr);
        }
    }

    public void diagRightWithEnc(double fwdpwr, double rhtpwr, int pulses, LinearOpMode opmode){
        if(fwdpwr > 0) {
            while (LF.getCurrentPosition() < pulses && opmode.opModeIsActive()) {
                LF.setPower(fwdpwr);
                LB.setPower(-rhtpwr);
                RF.setPower(-rhtpwr);
                RB.setPower(fwdpwr);
            }
        }else{
            while (LF.getCurrentPosition() > pulses && opmode.opModeIsActive()) {
                LF.setPower(fwdpwr);
                LB.setPower(-rhtpwr);
                RF.setPower(-rhtpwr);
                RB.setPower(fwdpwr);
            }
        }
    }

    public void diagLeftWithEnc(double fwdpwr, double lftpwr, int pulses, LinearOpMode opmode){
        if(fwdpwr > 0) {
            while (LF.getCurrentPosition() < pulses && opmode.opModeIsActive()) {
                LF.setPower(-lftpwr);
                LB.setPower(fwdpwr);
                RF.setPower(fwdpwr);
                RB.setPower(-lftpwr);
            }
        }else{
            while (LF.getCurrentPosition() > pulses && opmode.opModeIsActive()) {
                LF.setPower(-lftpwr);
                LB.setPower(fwdpwr);
                RF.setPower(fwdpwr);
                RB.setPower(-lftpwr);
            }
        }
    }

    public void primeServo(){
        frontYk.setPosition(0.7);
        backYk.setPosition(0.2);
    }

    public void grabFront(){
        frontYk.setPosition(0);
    }

    public void releaseFront(){
        frontYk.setPosition(0.7);
    }

    public void grabBack(){
        backYk.setPosition(1);
    }

    public void releaseBack(){
        backYk.setPosition(0.2);
    }

}
