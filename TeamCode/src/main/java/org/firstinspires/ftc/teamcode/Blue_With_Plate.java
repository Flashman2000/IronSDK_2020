package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TransitionSoftware.AutoTransitioner;
import org.firstinspires.ftc.teamcode.configs.Monmon;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous(name = "Test Blue Plate")
public class Blue_With_Plate extends LinearOpMode {

    String SkystoneLocation;

    @Override
    public void runOpMode(){

        SampleMecanumDriveREVOptimized robot = new SampleMecanumDriveREVOptimized(hardwareMap, true);

        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);

        AutoTransitioner.transitionOnStop(this, "Teleop");

        waitForStart();

        robot.releaseBack();
        robot.releaseFront();

        telemetry.clearAll();

        SkystoneLocation = robot.detectSkystone();

        telemetry.addData("Position", SkystoneLocation);
        telemetry.update();

        robot.alignSkystone(SkystoneLocation, this);
        robot.killAll();
        robot.resetEncoders();

        robot.closePipeline(); //avoid RAM leak
                                                                 
        if(SkystoneLocation == "Left" || SkystoneLocation == "NOT FOUND"){

            robot.strafeRightWithEnc(0.6, 1832, this);
            robot.killAll();
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 80, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            //robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 350, this); //sped up
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(85, this, telemetry);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.8, 4500, this);
            robot.releaseFront();
            //robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            robot.strafeRightWithEnc(0.68, 6000, this);
            robot.resetEncoders();
            robot.killAll();
            robot.resetEncoders();
            robot.resetEncoders();
            robot.bckWithTime(0.3, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.clampBack();
            sleep(1000);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(1, 850, this);
            robot.turnLeft180(1,this);
            robot.killAll();
            robot.resetEncoders();
            robot.unclampBack();
            sleep(1000);
            robot.fwdWithEncoderNoCorrect(0.6, 200, this);
            robot.resetEncoders();
            robot.turnRightEnc(0.6, 600, this); //sped up
            robot.turnRightGyro(90, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeLeftWithEnc(1, 5550, this);
            robot.reOrient(this);
            robot.killAll();
            robot.setMaintainedHeading(0);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            //robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 750, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(85, this, telemetry);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(1, 4500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            //robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            sleep(1000);
            robot.strafeLeftWithEnc(1, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeft180(0.6, this);
            robot.killAll();
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(0);
            sleep(2000);

        }

        if(SkystoneLocation == "Center"){

            robot.strafeRightWithEnc(0.6, 1832, this);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(0.6, 80, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 350, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(80, this, telemetry);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.8, 4500, this);
            robot.releaseFront();
            robot.strafeRightWithEnc(0.68, 6000, this);
            robot.resetEncoders();
            robot.setMaintainedHeading(90);
            robot.killAll();
            robot.resetEncoders();
            robot.resetEncoders();
            robot.bckWithTime(0.3, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.clampBack();
            sleep(1000);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(1, 850, this);
            robot.turnLeft180(1,this);
            robot.killAll();
            robot.resetEncoders();
            robot.unclampBack();
            sleep(1000);
            robot.fwdWithEncoderNoCorrect(0.6, 200, this);
            robot.resetEncoders();
            robot.turnRightEnc(0.6, 600, this);
            robot.turnRightGyro(90, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeLeftWithEnc(0.9, 6250, this);
            robot.reOrient(this);
            robot.killAll();
            robot.setMaintainedHeading(0);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 400, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(80, this,telemetry);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(1, 4500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(1000);
            robot.strafeLeftWithEnc(1, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeft180(0.6,this);
            robot.killAll();
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(0);
            sleep(2000);

        }

        if(SkystoneLocation == "Right"){

            robot.strafeRightWithEnc(0.6, 1832, this);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(0.6, 120, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 420, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(80, this,telemetry);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.8, 4500, this);
            robot.releaseBack();
            robot.strafeRightWithEnc(0.68, 6080, this);
            robot.resetEncoders();
            robot.setMaintainedHeading(90);
            robot.killAll();
            robot.resetEncoders();
            robot.resetEncoders();
            robot.bckWithTime(0.3, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.clampBack();
            sleep(1000);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(1, 850, this);
            robot.turnLeft180(1,this);
            robot.killAll();
            robot.resetEncoders();
            robot.unclampBack();
            sleep(1000);
            robot.fwdWithEncoderNoCorrect(0.6, 200, this);
            robot.resetEncoders();
            robot.turnRightEnc(0.6, 600, this);
            robot.turnRightGyro(90, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeLeftWithEnc(0.9, 6750, this);
            robot.reOrient(this);
            robot.killAll();
            robot.setMaintainedHeading(0);
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 400, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(80, this, telemetry);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(1, 4500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(1000);
            robot.strafeLeftWithEnc(1, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeft180(0.6, this);
            robot.killAll();
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(0);
            sleep(2000);

        }

    }
}

