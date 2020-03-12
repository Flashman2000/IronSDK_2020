package org.firstinspires.ftc.teamcode.drive.mecanum;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    public CRServo tapeMeasure;
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
        if(auto) {
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        }if(!auto){
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        }
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

        tapeMeasure = hardwareMap.get(CRServo.class, "tm");

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
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

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
        RB.setPower(v2*0.95);
        RF.setPower(v3*0.95);
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

    public void poseCorrect(SampleMecanumDriveREVOptimized robot, double goalX, double goalY){

        double offset = robot.imu.getAngularOrientation().firstAngle;
        robot.turnSync(-offset, robot.RF, robot.LB);

        Pose2d currPose = robot.getPoseEstimate();
        double currX = currPose.getX();
        double currY = currPose.getY();
        double errorX = currX-goalX; //if negative, the robot has undershot. if positive, robot has overshot
        double errorY = currY-goalY; //if negative, too much to the right, if positive, too much to the left

        Trajectory traj = robot.trajectoryBuilder()
                .forward(Math.abs(0))
                .strafeLeft(Math.abs(0))
                .build();

        if(errorX < 0 && errorY < 0){
            traj = robot.trajectoryBuilder()
                    .forward(Math.abs(errorX))
                    .strafeLeft(Math.abs(errorY))
                    .build();
        }

        if(errorX < 0 && errorY > 0){
            traj = robot.trajectoryBuilder()
                    .forward(Math.abs(errorX))
                    .strafeRight(Math.abs(errorY))
                    .build();
        }

        if(errorX > 0 && errorY < 0){
            traj = robot.trajectoryBuilder()
                    .back(Math.abs(errorX))
                    .strafeLeft(Math.abs(errorY))
                    .build();
        }

        if(errorX > 0 && errorY > 0){
            traj = robot.trajectoryBuilder()
                    .back(Math.abs(errorX))
                    .strafeRight(Math.abs(errorY))
                    .build();
        }

        robot.followTrajectorySync(traj);

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

    public void fwd(double pwr){
        correction = checkDirection(GAIN);
        LF.setPower(pwr-correction);
        LB.setPower(pwr+correction);
        RF.setPower(pwr-correction);
        RB.setPower(pwr+correction);
    }

    public void bck (double pwr){
        correction = checkDirection(GAIN);
        LF.setPower(-pwr-correction);
        LB.setPower(-pwr+correction);
        RF.setPower(-pwr-correction);
        RB.setPower(-pwr+correction);
    }

    public void strafeRight(double pwr){

        correction = checkDirection(GAIN);
        LF.setPower(pwr-correction);
        LB.setPower(-pwr+correction);
        RF.setPower(-pwr-correction);
        RB.setPower(pwr+correction);

    }

    public void strafeLeft(double pwr){

        correction = checkDirection(GAIN);
        LF.setPower(-pwr-correction);
        LB.setPower(pwr+correction);
        RF.setPower(pwr-correction);
        RB.setPower(-pwr+correction);

    }

    public void turnLeft(double pwr){
        LF.setPower(-pwr);
        LB.setPower(pwr);
        RF.setPower(-pwr);
        RB.setPower(pwr);
    }

    public void strafeLeftNC(double pwr){
        LF.setPower(-pwr);
        LB.setPower(pwr);
        RF.setPower(pwr);
        RB.setPower(-pwr);
    }

    public void fwdNC(double pwr){
        LF.setPower(pwr);
        LB.setPower(pwr);
        RF.setPower(pwr);
        RB.setPower(pwr);
    }

    public void backWithOdo(ExpansionHubMotor leftOdo, ExpansionHubMotor rightOdo, double target, double oldPosL, double oldPosR, LinearOpMode opmode){

        while (leftOdo.getCurrentPosition() > -target + oldPosL && rightOdo.getCurrentPosition() > -target + oldPosR && opmode.opModeIsActive()){
            bck(0.8);
            if(leftOdo.getCurrentPosition() < -target + 6000 + oldPosL && rightOdo.getCurrentPosition() > -target + 3000 + oldPosR){
                bck(0.5);
            }
        }

    }

    public void fwdWithOdo(ExpansionHubMotor leftOdo, ExpansionHubMotor rightOdo, double target, double oldPosL, double oldPosR, LinearOpMode opmode){

        while (leftOdo.getCurrentPosition() < target + oldPosL && rightOdo.getCurrentPosition() > -target + oldPosR && opmode.opModeIsActive()){
            fwd(0.8);
            if(leftOdo.getCurrentPosition() > target - 6000 + oldPosL && rightOdo.getCurrentPosition() > -target + 3000 + oldPosR){
                fwd(0.5);
            }
        }

    }

    public void fwdWithOdoNC(ExpansionHubMotor leftOdo, ExpansionHubMotor rightOdo, double target, double oldPosL, double oldPosR, LinearOpMode opmode){

        while (leftOdo.getCurrentPosition() < target + oldPosL && rightOdo.getCurrentPosition() > -target + oldPosR && opmode.opModeIsActive()){
            fwdNC(0.8);
            if(leftOdo.getCurrentPosition() > target - 6000 + oldPosL && rightOdo.getCurrentPosition() > -target + 3000 + oldPosR){
                fwdNC(0.5);
            }
        }

    }

    public void strafeRightWithOdo(ExpansionHubMotor backOdo, double target, double oldPosB, LinearOpMode opmode){

        while (backOdo.getCurrentPosition() > -target + oldPosB && opmode.opModeIsActive()){
            strafeRight(0.8);
            if(backOdo.getCurrentPosition() < -target + oldPosB + 6000){
                strafeRight(0.5);
            }
        }

    }

    public void strafeLeftWithOdo(ExpansionHubMotor backOdo, double target, double oldPosB, LinearOpMode opmode){

        while (backOdo.getCurrentPosition() < target + oldPosB && opmode.opModeIsActive()){
            strafeLeft(0.8);
            if(backOdo.getCurrentPosition() > target + oldPosB - 6000){
                strafeLeft(0.5);
            }
        }

    }

    public void strafeLeftWithOdoNC(ExpansionHubMotor backOdo, double target, double oldPosB, LinearOpMode opmode){

        while (backOdo.getCurrentPosition() < target + oldPosB && opmode.opModeIsActive()){
            strafeLeftNC(0.8);
            if(backOdo.getCurrentPosition() > target + oldPosB - 6000){
                strafeLeftNC(0.5);
            }
        }

    }



    public void turnLeftWithOdo(ExpansionHubMotor backodo, double target, double oldPosB, LinearOpMode opmode){

        while (backodo.getCurrentPosition() > - target + oldPosB && opmode.opModeIsActive()){
            turnLeft(0.8);
            if(backodo.getCurrentPosition() < - target + oldPosB + 6000){
                turnLeft(0.35);
            }
        }

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

    public void relayPose(Telemetry telemetry, SampleMecanumDriveREVOptimized robot){

        telemetry.addData("X", robot.getPoseEstimate().getX());
        telemetry.addData("Y", robot.getPoseEstimate().getY());
        telemetry.addData("Heading", robot.getPoseEstimate().getHeading());
        telemetry.update();

    }

}
