package org.firstinspires.ftc.teamcode.drive.mecanum;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.IronCV.IronCVDetectorClass;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.MecanumPowers;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    public ExpansionHubEx hub;
    public ExpansionHubEx hub2;
    public ExpansionHubMotor LF, LB, RB, RF;
    public DcMotor lColl, rColl, spool, spool2;
    public Servo backs, leftArm, rightArm, grabber, turner, frontYkA, backYkA, frontYk, backYk, cap;
    public RevBlinkinLedDriver blinkinLedDriver;
    private List<ExpansionHubMotor> driveMotors;
    public BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    public double maintainedHeading = 0;
    private MecanumPowers powers;

    public IronCVDetectorClass detector = new IronCVDetectorClass();

    public SampleMecanumDriveREVOptimized(HardwareMap hardwareMap, boolean auto) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        hub = hardwareMap.get(ExpansionHubEx.class, "hub");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "hub2");

        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        /**
         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
         parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
         parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
         parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
         parameters.loggingEnabled      = true;
         parameters.loggingTag          = "IMU";
         parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

         imu = hardwareMap.get(BNO055IMU.class, "imu 1");
         parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
         imu.initialize(parameters);
         */

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        //BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        LF = hardwareMap.get(ExpansionHubMotor.class, "lf");
        LB = hardwareMap.get(ExpansionHubMotor.class, "lb");
        RB = hardwareMap.get(ExpansionHubMotor.class, "rb");
        RF = hardwareMap.get(ExpansionHubMotor.class, "rf");

        lColl = hardwareMap.get(DcMotor.class, "lc");
        rColl = hardwareMap.get(DcMotor.class, "rc");
        spool = hardwareMap.get(DcMotor.class, "spool");
        spool2 = hardwareMap.get(DcMotor.class, "spool2");

        backs = hardwareMap.get(ExpansionHubServo.class, "backs");
        leftArm = hardwareMap.get(ExpansionHubServo.class, "lcoll");
        rightArm = hardwareMap.get(ExpansionHubServo.class, "rcoll");

        grabber = hardwareMap.get(ExpansionHubServo.class, "grab");
        turner = hardwareMap.get(ExpansionHubServo.class, "turn");
        cap = hardwareMap.get(ExpansionHubServo.class, "cap");
        //spacer = hardwareMap.get(ExpansionHubServo.class, "spacer");

        frontYk = hardwareMap.get(ExpansionHubServo.class, "frntyk");
        backYk = hardwareMap.get(ExpansionHubServo.class, "bckyk");
        frontYkA = hardwareMap.get(ExpansionHubServo.class, "frntyka");
        backYkA = hardwareMap.get(ExpansionHubServo.class, "backyka");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        driveMotors = Arrays.asList(LF, LB, RB, RF);

            for (ExpansionHubMotor motor : driveMotors) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }


        rColl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lColl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spool2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);

        StandardThreeWheelLocalizer localizer = new StandardThreeWheelLocalizer(hardwareMap);

        setLocalizer(localizer);

        if(auto) {
            detector.camSetup(hardwareMap);
        }
    }


    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = LF.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : driveMotors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : driveMotors) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelVelocities = new ArrayList<>();
        for (ExpansionHubMotor motor : driveMotors) {
            wheelVelocities.add(encoderTicksToInches(bulkData.getMotorVelocity(motor)));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        LF.setPower(v);
        LB.setPower(v1);
        RB.setPower(v2*0.99);
        RF.setPower(v3*0.99);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }


    public void setPowers(MecanumPowers powers) {
        this.powers = powers;
        LF.setPower(powers.frontLeft);
        RF.setPower(powers.frontRight*0.95);
        LB.setPower(powers.backLeft);
        RB.setPower(powers.backRight*0.95);
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

        if(gamepad1.a){
            backs.setPosition(0);
        }
        if(gamepad1.b){
            backs.setPosition(1);
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

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void fwd(double pwr){
        //correction = checkDirection(GAIN);
        LF.setPower(pwr);
        LB.setPower(pwr);
        RF.setPower(pwr);
        RB.setPower(pwr);
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

    public void bck (double pwr){
        correction = checkDirection(GAIN);
        LF.setPower(-pwr - correction);
        LB.setPower(-pwr + correction);
        RF.setPower(-pwr - correction);
        RB.setPower(-pwr + correction);
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

    public void strafeRight(double pwr){

        //correction = checkDirection(GAIN);
        LF.setPower(pwr);
        LB.setPower(-pwr);
        RF.setPower(-pwr);
        RB.setPower(pwr);

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

    public void strafeLeft(double pwr){
        correction = checkDirection(GAIN);
        LF.setPower(-pwr - correction);
        LB.setPower(pwr + correction);
        RF.setPower(pwr - correction);
        RB.setPower(-pwr + correction);
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
        frontYk.setPosition(0.2);
        backYk.setPosition(0.7);
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
        backs.setPosition(0.6);
    }

    public void clampBack(){
        backs.setPosition(0);
    }

    public void unclampBack(){
        backs.setPosition(1);
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
            angle = maintainedHeading - imu.getAngularOrientation().firstAngle;
        }

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void turnLeftGyro(double target, LinearOpMode opmode, Telemetry telemetry) {

        if(target > 0) {
            while (imu.getAngularOrientation().firstAngle < target && opmode.opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                double pwr = Range.clip(Math.abs(0.011*(imu.getAngularOrientation().firstAngle - target)), 0.3, 1);
                LF.setPower(-pwr);
                LB.setPower(pwr);
                RF.setPower(-pwr);
                RB.setPower(pwr);
                telemetry.addData("Heading", imu.getAngularOrientation().firstAngle);
                telemetry.update();
            }
            killAll();
        }
        if(target < 0){
            while(imu.getAngularOrientation().firstAngle < target && opmode.opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                double pwr = Range.clip(Math.abs(0.011*(imu.getAngularOrientation().firstAngle - target)), 0.3, 1);
                LF.setPower(-pwr);
                LB.setPower(pwr);
                RF.setPower(-pwr);
                RB.setPower(pwr);
            }
            killAll();
        }

        /*
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
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                double pwr = Range.clip(Math.abs(0.011*(imu.getAngularOrientation().firstAngle-target)), 0.3, 1);
                LF.setPower(pwr);
                LB.setPower(-pwr);
                RF.setPower(pwr);
                RB.setPower(-pwr);
            }
            killAll();
        }
        if(target < 0){
            while (imu.getAngularOrientation().firstAngle > target && opmode.opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                double pwr = Range.clip(Math.abs(0.011*(imu.getAngularOrientation().firstAngle-target)), 0.3, 1);
                LF.setPower(pwr);
                LB.setPower(-pwr);
                RF.setPower(pwr);
                RB.setPower(-pwr);
            }
            killAll();
        }

        /*
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

    public void turnLeft180(double pwr, LinearOpMode opmode){
        boolean positive = true;
        while (imu.getAngularOrientation().firstAngle != 180 && positive && opmode.opModeIsActive()){

            if(imu.getAngularOrientation().firstAngle >= -180 && imu.getAngularOrientation().firstAngle < 0){
                positive = false;
            }

            LF.setPower(-pwr);
            LB.setPower(pwr);
            RF.setPower(-pwr);
            RB.setPower(pwr);
        }
        killAll();

    }

    public void turnRight180(double pwr, LinearOpMode opmode){
        boolean positive = true;
        while (imu.getAngularOrientation().firstAngle != 180 && positive && opmode.opModeIsActive()){

            if(imu.getAngularOrientation().firstAngle >= -180 && imu.getAngularOrientation().firstAngle < 0){
                positive = false;
            }

            LF.setPower(pwr);
            LB.setPower(-pwr);
            RF.setPower(pwr);
            RB.setPower(-pwr);
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

    public void turnTo(double angle) {
        angle -= Math.toDegrees(getExternalHeading());
        while (angle < -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
        turnSync(Math.toRadians(angle), RF, LB);
    }

    public void alignSkystone(String orientation, LinearOpMode opmde){

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
            resetEncoders();
            bckWithEncoder(0.6, 80, opmde);
            killAll();
            resetEncoders();

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
            resetEncoders();
            fwdWithEncoder(0.6, 80, opmde);
            killAll();
            resetEncoders();
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
            resetEncoders();
            fwdWithEncoder(0.6, 120, opmde);
            killAll();
            resetEncoders();
        }

    }

    public void moveGrab(String status, LinearOpMode opmode){

        if(status == "Grab"){
            backYkA.setPosition(1);
            opmode.sleep(200);
            backYk.setPosition(0);
            opmode.sleep(200);
            backYkA.setPosition(0.5);
        }

        if(status == "Drop"){
            backYkA.setPosition(1);
            opmode.sleep(200);
            backYk.setPosition(0);
            opmode.sleep(200);
            backYkA.setPosition(0.5);
        }

    }

    public void relayPose(Telemetry telemetry, SampleMecanumDriveREVOptimized robot){

        telemetry.addData("X", robot.getPoseEstimate().getX());
        telemetry.addData("Y", robot.getPoseEstimate().getY());
        telemetry.addData("Heading", robot.getPoseEstimate().getHeading());
        telemetry.update();

    }

}
