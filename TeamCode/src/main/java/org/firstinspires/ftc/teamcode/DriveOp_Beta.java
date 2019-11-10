package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.opencv.core.Mat;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

@TeleOp(name = "DRIVE")
public class DriveOp_Beta extends LinearOpMode {

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

    BNO055IMU imu;

    @Override
    public void runOpMode(){

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


        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        rColl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lColl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()){

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
                backL.setPosition(0);
                backR.setPosition(0);
            }
            if(gamepad1.b){
                backL.setPosition(1);
                backR.setPosition(1);
            }

            if(gamepad1.dpad_up){

                leftArm.setPosition(0);
                rightArm.setPosition(1);

            }

            if(gamepad1.dpad_down){

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

            if (gamepad2.a){

                grabber.setPosition(1);

            }

            if(gamepad2.b){

                grabber.setPosition(0);

            }

            if(gamepad2.dpad_left){
                turner.setPosition(1);
            }

            if (gamepad2.dpad_right){
                turner.setPosition(0);
            }


        }

    }

}
