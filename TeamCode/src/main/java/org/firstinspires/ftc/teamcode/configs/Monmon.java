package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Monmon extends Monmon_Config{
    public double maintainedHeading = 0;

    public Monmon(){ }

    public void initAuto(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opmode, boolean camEnable){

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

        primeBack();

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

        turner.setPosition(1);
        i = turner.getPosition();
    }

    public void teleActivity(Gamepad gamepad1, Gamepad gamepad2){
        releaseBack();
        releaseFront();

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
            lColl.setPower(0.3);
            rColl.setPower(-0.3);
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

    public void closePipeline(){
        detector.webcam.stopStreaming();
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

    public void turnRightEnc(double pwr, int pulses, LinearOpMode opmode){
        while(LF.getCurrentPosition() < pulses && opmode.opModeIsActive()) {
            LF.setPower(pwr);
            LB.setPower(-pwr);
            RF.setPower(pwr);
            RB.setPower(-pwr);
        }
    }

    public void turnLeftEnc(double pwr, int pulses, LinearOpMode opmode){
        while(LF.getCurrentPosition() > -pulses && opmode.opModeIsActive()) {
            LF.setPower(-pwr);
            LB.setPower(pwr);
            RF.setPower(-pwr);
            RB.setPower(pwr);
        }
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

    public void fwdWithEncoderNoCorrect(double pwr, int pulses, LinearOpMode opmode){
        while(LF.getCurrentPosition() < pulses && opmode.opModeIsActive()){
            LF.setPower(pwr);
            LB.setPower(pwr);
            RF.setPower(pwr);
            RB.setPower(pwr);
        }
    }

    public void fwdWithTime(double pwr, long time, LinearOpMode opmode){
        correction = checkDirection(GAIN);
        LF.setPower(pwr - correction);
        LB.setPower(pwr + correction);
        RF.setPower(pwr - correction);
        RB.setPower(pwr + correction);
        opmode.sleep(time);
        killAll();
    }

    public void bckWithEncoder(double pwr, int pulses, LinearOpMode opmode){
        while(LF.getCurrentPosition() > -pulses && opmode.opModeIsActive()){
            correction = checkDirection(GAIN);
            LF.setPower(-pwr - correction);
            LB.setPower(-pwr + correction);
            RF.setPower(-pwr - correction);
            RB.setPower(-pwr + correction);
        }
    }

    public void bckWithTime(double pwr, long time, LinearOpMode opmode){
        correction = checkDirection(GAIN);
        LF.setPower(-pwr - correction);
        LB.setPower(-pwr + correction);
        RF.setPower(-pwr - correction);
        RB.setPower(-pwr + correction);
        opmode.sleep(time);
    }

    public void strafeLeftWithEnc(double pwr, int pulses, LinearOpMode opmode){
        while(LF.getCurrentPosition() > -pulses && opmode.opModeIsActive()){
            correction = checkDirection(GAIN);
            LF.setPower(-pwr - correction);
            LB.setPower(pwr + correction);
            RF.setPower(pwr - correction);
            RB.setPower(-pwr + correction);
        }
    }

    public void strafeRightWithEnc(double pwr, int pulses, LinearOpMode opmode){
        while(LF.getCurrentPosition() < pulses && opmode.opModeIsActive()){
            correction = checkDirection(GAIN);
            LF.setPower(pwr - correction);
            LB.setPower(-pwr + correction);
            RF.setPower(-pwr - correction);
            RB.setPower(pwr + correction);
        }
    }

    public void strafeRightWithTime(double pwr, long time, LinearOpMode opmode){
        correction = checkDirection(GAIN);
        LF.setPower(pwr - correction);
        LB.setPower(-pwr + correction);
        RF.setPower(-pwr - correction);
        RB.setPower(pwr + correction);
        opmode.sleep(time);
        killAll();
    }

    public void strafeLeftWithTime(double pwr, long time, LinearOpMode opmode){
        correction = checkDirection(GAIN);
        LF.setPower(-pwr - correction);
        LB.setPower(pwr + correction);
        RF.setPower(pwr - correction);
        RB.setPower(-pwr + correction);
        opmode.sleep(time);
        killAll();
    }

    public void diagRightWithEnc(double fwdpwr, double rhtpwr, int pulses, LinearOpMode opmode){
        if(fwdpwr > 0) {
            while (LF.getCurrentPosition() < pulses && opmode.opModeIsActive()) {
                correction = checkDirection(GAIN);
                LF.setPower(fwdpwr - correction);
                LB.setPower(-rhtpwr + correction);
                RF.setPower(-rhtpwr - correction);
                RB.setPower(fwdpwr + correction);
            }
        }else{
            while (LF.getCurrentPosition() > pulses && opmode.opModeIsActive()) {
                correction = checkDirection(GAIN);
                LF.setPower(fwdpwr - correction);
                LB.setPower(-rhtpwr + correction);
                RF.setPower(-rhtpwr - correction);
                RB.setPower(fwdpwr + correction);
            }
        }
    }

    public void diagLeftWithEnc(double fwdpwr, double lftpwr, int pulses, LinearOpMode opmode){
        if(fwdpwr > 0) {
            while (LF.getCurrentPosition() > -pulses && opmode.opModeIsActive()) {
                correction = checkDirection(GAIN);
                LF.setPower(-lftpwr - correction);
                LB.setPower(fwdpwr + correction);
                RF.setPower(fwdpwr - correction);
                RB.setPower(-lftpwr + correction);
            }
        }else{
            while (LF.getCurrentPosition() < -pulses && opmode.opModeIsActive()) {
                correction = checkDirection(GAIN);
                LF.setPower(-lftpwr - correction);
                LB.setPower(fwdpwr + correction);
                RF.setPower(fwdpwr - correction);
                RB.setPower(-lftpwr + correction);
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

    public void primeBack(){
        backL.setPosition(0.5);
        backR.setPosition(0.5);
    }

    public void clampBack(){
        backL.setPosition(0);
        backR.setPosition(0);
    }

    public void unclampBack(){
        backL.setPosition(1);
        backR.setPosition(1);
    }

    public void setMaintainedHeading(double heading){
        maintainedHeading = heading;
    }

    public double checkDirection(double adj) {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle=0, gain = adj;

        if(maintainedHeading >= 0) {
            angle = imu.getAngularOrientation().firstAngle - maintainedHeading;
        }else if(maintainedHeading < 0){
            angle = -maintainedHeading - imu.getAngularOrientation().firstAngle;
        }

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void turnLeftGyro(double target, LinearOpMode opmode) {

        if(target > 0) {
            while (imu.getAngularOrientation().firstAngle < target && opmode.opModeIsActive()) {
                double pwr = Range.clip((target - imu.getAngularOrientation().firstAngle) / target, 0.3, 1);
                LF.setPower(-pwr);
                LB.setPower(pwr);
                RF.setPower(-pwr);
                RB.setPower(pwr);
            }
            killAll();
        }
        if(target < 0){
            while(imu.getAngularOrientation().firstAngle < target && opmode.opModeIsActive()) {
                double pwr = Range.clip((target - imu.getAngularOrientation().firstAngle) / target, -1, -0.3);
                LF.setPower(pwr);
                LB.setPower(-pwr);
                RF.setPower(pwr);
                RB.setPower(-pwr);
            }
            killAll();
        }

        /**
        if (target > 0){
            while (imu.getAngularOrientation().firstAngle < target && opmode.opModeIsActive()) {
                double pwrcalc = Math.pow(target, 2) - Math.pow(imu.getAngularOrientation().firstAngle, 2) / Math.pow(target, 2);
                double pwr = Range.clip(pwrcalc, 0.1, 0.6);
                LF.setPower(-pwr);
                LB.setPower(pwr);
                RF.setPower(-pwr);
                RB.setPower(pwr);
            }
            killAll();
        }
        if(target < 0){
            while(imu.getAngularOrientation().firstAngle < target && opmode.opModeIsActive()){
                double pwrcalc = Math.pow(target, 2) - Math.pow(imu.getAngularOrientation().firstAngle, 2) / Math.pow(target, 2);
                double pwr = Range.clip(pwrcalc, -0.6, -0.1);
                LF.setPower(pwr);
                LB.setPower(-pwr);
                RF.setPower(pwr);
                RB.setPower(-pwr);
            }
            killAll();
        }
         */
    }

    public void turnRightGyro(double target, LinearOpMode opmode){

        if(target > 0) {
            while (imu.getAngularOrientation().firstAngle > target && opmode.opModeIsActive()) {
                double pwr = Range.clip((target - imu.getAngularOrientation().firstAngle) / target, 0.3, 1);
                LF.setPower(pwr);
                LB.setPower(-pwr);
                RF.setPower(pwr);
                RB.setPower(-pwr);
            }
            killAll();
        }
        if(target < 0){
            while (imu.getAngularOrientation().firstAngle > target && opmode.opModeIsActive()) {
                double pwr = Range.clip((target - imu.getAngularOrientation().firstAngle) / target, -1, -0.3);
                LF.setPower(-pwr);
                LB.setPower(pwr);
                RF.setPower(-pwr);
                RB.setPower(pwr);
            }
            killAll();
        }

        /**
        if (target > 0){
            while (imu.getAngularOrientation().firstAngle > target && opmode.opModeIsActive()) {
                double pwrcalc = Math.pow(target, 2) - Math.pow(imu.getAngularOrientation().firstAngle, 2) / Math.pow(target, 2);
                double pwr = Range.clip(pwrcalc, 0.1, 0.6);
                LF.setPower(pwr);
                LB.setPower(-pwr);
                RF.setPower(pwr);
                RB.setPower(-pwr);
            }
            killAll();
        }
        if(target < 0){
            while(imu.getAngularOrientation().firstAngle > target && opmode.opModeIsActive()){
                double pwrcalc = Math.pow(target, 2) - Math.pow(imu.getAngularOrientation().firstAngle, 2) / Math.pow(target, 2);
                double pwr = Range.clip(pwrcalc, -0.6, -0.1);
                LF.setPower(-pwr);
                LB.setPower(pwr);
                RF.setPower(-pwr);
                RB.setPower(pwr);
            }
            killAll();
        }
         */

    }

    public void turnLeft180(LinearOpMode opmode){
        boolean positive = true;
        while (imu.getAngularOrientation().firstAngle != 180 && positive && opmode.opModeIsActive()){

            if(imu.getAngularOrientation().firstAngle >= -180 && imu.getAngularOrientation().firstAngle < 0){
                positive = false;
            }

            LF.setPower(-1);
            LB.setPower(1);
            RF.setPower(-1);
            RB.setPower(1);
        }
        killAll();

    }

    public void turnRight180(LinearOpMode opmode){
        boolean negative = true;
        while (imu.getAngularOrientation().firstAngle != -180 && negative && opmode.opModeIsActive()){

            if(imu.getAngularOrientation().firstAngle <= 180 && imu.getAngularOrientation().firstAngle > 0){
                negative = false;
            }

            LF.setPower(1);
            LB.setPower(-1);
            RF.setPower(1);
            RB.setPower(-1);
        }
        killAll();
    }

    public void reOrient(LinearOpMode opmode){
        if(imu.getAngularOrientation().firstAngle > 0){
            while(imu.getAngularOrientation().firstAngle > 0 && opmode.opModeIsActive()) {
                double pwr = Range.clip(Math.abs(0.011*imu.getAngularOrientation().firstAngle), 0.3, 1);
                LF.setPower(pwr);
                LB.setPower(-pwr);
                RF.setPower(pwr);
                RB.setPower(-pwr);
            }
            killAll();
        }else if(imu.getAngularOrientation().firstAngle < 0){
            while(imu.getAngularOrientation().firstAngle < 0 && opmode.opModeIsActive()) {
                double pwr = Range.clip(Math.abs(0.011*imu.getAngularOrientation().firstAngle), 0.3, 1);
                LF.setPower(-pwr);
                LB.setPower(pwr);
                RF.setPower(-pwr);
                RB.setPower(pwr);
            }
            killAll();
        }
    }

    public void alignSkystone(String orientation){

        //vals are stored in 1, 0, 2 from left to right

        detector.updateVals();
        vals = detector.getVals();

        if(orientation == "Left" || orientation == "NOT FOUND"){
            while(vals[0] > 210){
                detector.updateVals();
                vals = detector.getVals();
                correction = checkDirection(GAIN);
                LF.setPower(0.25);
                LB.setPower(0.25);
                RF.setPower(0.25);
                RB.setPower(0.25);
            }
            killAll();

        }

        if(orientation == "Center"){
            while(vals[1] > 210){
                detector.updateVals();
                vals = detector.getVals();
                correction = checkDirection(GAIN);
                LF.setPower(-0.25);
                LB.setPower(-0.25);
                RF.setPower(-0.25);
                RB.setPower(-0.25);
            }
            killAll();
        }

        if(orientation == "Right"){
            while(vals[0] > 210){
                detector.updateVals();
                vals = detector.getVals();
                correction = checkDirection(GAIN);
                LF.setPower(-0.25);
                LB.setPower(-0.25);
                RF.setPower(-0.25);
                RB.setPower(-0.25);
            }
            killAll();
        }

    }

}
