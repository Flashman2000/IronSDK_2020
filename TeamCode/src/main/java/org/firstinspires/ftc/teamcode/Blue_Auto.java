package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
public class Blue_Auto extends LinearOpMode {

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

    Orientation angles;

    IronCVDetectorClass detector = new IronCVDetectorClass(1f/8f, 3f/8f);

    String pos;

    int[] vals;

    @Override
    public void runOpMode() {

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

        leftArm.setPosition(0);
        rightArm.setPosition(1);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        waitForStart();

        detector.updateVals();
        vals = detector.getVals();

        if(vals[1] < 200 && vals[0] > 210 && vals[2] > 210){
            pos = "Left";
        }else if(vals[1] > 210 && vals[0] < 200 && vals[2] > 210){
            pos = "Center";
        }else if(vals[1] > 210 && vals[0] > 210 && vals[2] < 200){
            pos = "Right";
        }else{
            pos = "NOT FOUND";
        }

        telemetry.addData("Position", pos);
        telemetry.update();

        detector.webcam.stopStreaming();

        sleep(100);

        leftArm.setPosition(1);
        rightArm.setPosition(0);

        sleep(200);


        rColl.setPower(0.5);
        lColl.setPower(-0.5);

        if(pos == "Center"){

            while(LF.getCurrentPosition() < 1100){

                LF.setPower(0.5);
                LB.setPower(0.5);
                RF.setPower(0.5);
                RB.setPower(0.5);

            }

            while(LF.getCurrentPosition() < 2500){

                LF.setPower(0.1);
                LB.setPower(0.1);
                RF.setPower(0.1);
                RB.setPower(0.1);

            }

        }

        if(pos == "Left"){

            while(LF.getCurrentPosition() > -550){

                LF.setPower(-0.25);
                LB.setPower(0.25);
                RF.setPower(0.25);
                RB.setPower(-0.25);

            }

            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while(LF.getCurrentPosition() < 1100){

                LF.setPower(0.5);
                LB.setPower(0.5);
                RF.setPower(0.5);
                RB.setPower(0.5);

            }

            while(LF.getCurrentPosition() < 2500){

                LF.setPower(0.1);
                LB.setPower(0.1);
                RF.setPower(0.1);
                RB.setPower(0.1);

            }

        }

        if(pos == "Right"){

            while(LF.getCurrentPosition() < 420){

                LF.setPower(0.25);
                LB.setPower(-0.25);
                RF.setPower(-0.25);
                RB.setPower(0.25);

            }

            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while(LF.getCurrentPosition() < 1100){

                LF.setPower(0.5);
                LB.setPower(0.5);
                RF.setPower(0.5);
                RB.setPower(0.5);

            }

            while(LF.getCurrentPosition() < 2500){

                LF.setPower(0.1);
                LB.setPower(0.1);
                RF.setPower(0.1);
                RB.setPower(0.1);

            }

        }

        grabber.setPosition(1);

        sleep(500);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(LF.getCurrentPosition() > -1150){

            LF.setPower(-0.3);
            LB.setPower(-0.3);
            RF.setPower(-0.3);
            RB.setPower(-0.3);

        }

        spool.setPower(-0.4);
        sleep(250);
        spool.setPower(0);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        if(pos == "Center") {
            while (LF.getCurrentPosition() > -3500) {

                LF.setPower(-1);
                LB.setPower(1);
                RF.setPower(1);
                RB.setPower(-1);

            }
        }

        if(pos == "Left"){
            while (LF.getCurrentPosition() > -2950) {

                LF.setPower(-1);
                LB.setPower(1);
                RF.setPower(1);
                RB.setPower(-1);

            }
        }

        if(pos == "Right"){
            while (LF.getCurrentPosition() > -3920) {

                LF.setPower(-1);
                LB.setPower(1);
                RF.setPower(1);
                RB.setPower(-1);

            }
        }

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lColl.setPower(0);
        rColl.setPower(0);

        while(LF.getCurrentPosition() > -1600){

            LF.setPower(-1);
            LB.setPower(1);
            RF.setPower(-1);
            RB.setPower(1);

        }

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);

        grabber.setPosition(0);
        sleep(500);
        leftArm.setPosition(0);
        rightArm.setPosition(1);
        sleep(1000);


        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(LF.getCurrentPosition() > -400){

            LF.setPower(-1);
            LB.setPower(-1);
            RF.setPower(-1);
            RB.setPower(-1);

        }


    }

}