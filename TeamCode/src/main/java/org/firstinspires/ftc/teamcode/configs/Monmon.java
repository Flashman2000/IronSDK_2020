package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Monmon extends Monmon_Config{

    public Monmon(){ }

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

    public void initTele(HardwareMap hardwareMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
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
        spacer = hardwareMap.get(Servo.class, "spacer");


        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        rColl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lColl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turner.setPosition(1);
        i = turner.getPosition();
    }

    public void teleActivity(Gamepad gamepad1, Gamepad gamepad2){
        double r1 = gamepad1.left_stick_x;
        double r2 = -gamepad1.left_stick_y;
        double t = gamepad1.right_stick_x;

        double rfpwr = -r1 + r2 + t;
        double rbpwr =  r1 + r2 - t;
        double lfpwr =  r1 + r2 + t;
        double lbpwr = -r1 + r2 - t;
        double spoolpowr = gamepad2.right_trigger - gamepad2.left_trigger;

        RF.setPower(rfpwr);
        RB.setPower(rbpwr);
        LF.setPower(lfpwr);
        LB.setPower(lbpwr);
        spool.setPower(spoolpowr);

        if(gamepad1.left_stick_button){
            spacer.setPosition(0);
        }
        if(gamepad1.right_stick_button){
            spacer.setPosition(0.35);
        }

        if(gamepad1.a){
            backL.setPosition(0);
            backR.setPosition(0);
        }
        if(gamepad1.b){
            backL.setPosition(1);
            backR.setPosition(1);
        }

        if(gamepad1.y){

            leftArm.setPosition(0);
            rightArm.setPosition(1);

        }

        if(gamepad1.x){

            leftArm.setPosition(1);
            rightArm.setPosition(0);

        }

        if(gamepad1.right_bumper){
            lColl.setPower(-0.7);
            rColl.setPower(0.7);
        }

        if(gamepad1.left_bumper){
            lColl.setPower(0);
            rColl.setPower(0);
            spacer.setPosition(0.35);
        }

        if(gamepad1.left_trigger > 0){
            lColl.setPower(1);
            rColl.setPower(-1);
        }

        if(gamepad1.dpad_left){
            RF.setPower(0.3);
            RB.setPower(-0.3);
            LF.setPower(-0.3);
            LB.setPower(0.3);
        }

        if(gamepad1.dpad_right){
            RF.setPower(-0.3);
            RB.setPower(0.3);
            LF.setPower(0.3);
            LB.setPower(-0.3);
        }

        if(gamepad1.dpad_up){
            RF.setPower(0.3);
            RB.setPower(0.3);
            LF.setPower(0.3);
            LB.setPower(0.3);
        }

        if(gamepad1.dpad_down){
            RF.setPower(-0.3);
            RB.setPower(-0.3);
            LF.setPower(-0.3);
            LB.setPower(-0.3);

        }

        if (gamepad2.a){

            grabber.setPosition(1);

        }

        if(gamepad2.b){

            grabber.setPosition(0);

        }
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
            correction = checkDirection(GAIN);
            LF.setPower(pwr - correction);
            LB.setPower(pwr + correction);
            RF.setPower(pwr - correction);
            RB.setPower(pwr + correction);
        }
    }

    public void fwdWithTime(double pwr, long time, LinearOpMode opmode){
        LF.setPower(pwr);
        LB.setPower(pwr);
        RF.setPower(pwr);
        RB.setPower(pwr);
        opmode.sleep(time);
        killAll();
    }

    public void bckWithEncoder(double pwr, int pulses, LinearOpMode opmode){
        while(LF.getCurrentPosition() > -pulses && opmode.opModeIsActive()){
            LF.setPower(-pwr);
            LB.setPower(-pwr);
            RF.setPower(-pwr);
            RB.setPower(-pwr);
        }
    }

    public void bckWithTime(double pwr, long time, LinearOpMode opmode){
        LF.setPower(-pwr);
        LB.setPower(-pwr);
        RF.setPower(-pwr);
        RB.setPower(-pwr);
        opmode.sleep(time);
        killAll();
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
            while (LF.getCurrentPosition() > -pulses && opmode.opModeIsActive()) {
                LF.setPower(-lftpwr);
                LB.setPower(fwdpwr);
                RF.setPower(fwdpwr);
                RB.setPower(-lftpwr);
            }
        }else{
            while (LF.getCurrentPosition() < -pulses && opmode.opModeIsActive()) {
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

    public void selfCorrect(){

        killAll();

        if(imu.getAngularOrientation().firstAngle != 0) {
            if (imu.getAngularOrientation().firstAngle > 0) {

                while (imu.getAngularOrientation().firstAngle >= 0) {
                    RF.setPower(0.25);
                    RB.setPower(-0.25);
                    LF.setPower(0.25);
                    LB.setPower(-0.25);
                }

                killAll();
                resetEncoders();

            } else if (imu.getAngularOrientation().firstAngle < 0) {

                while (imu.getAngularOrientation().firstAngle <= 0) {
                    RF.setPower(-0.25);
                    RB.setPower(0.25);
                    LF.setPower(-0.25);
                    LB.setPower(0.25);
                }

                killAll();
                resetEncoders();

            }
        }

    }

    public double checkDirection(double adj) {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = adj;

        angle = imu.getAngularOrientation().firstAngle;

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

}
