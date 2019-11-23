package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TransitionSoftware.AutoTransitioner;
import org.firstinspires.ftc.teamcode.configs.Monmon;

@Autonomous(name = "Test Red Plate")
public class Red_With_Plate extends LinearOpMode {

    Monmon robot = new Monmon();
    String SkystoneLocation;

    @Override
    public void runOpMode(){

        robot.initAuto(hardwareMap, telemetry, this, true);

        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);

        AutoTransitioner.transitionOnStop(this, "Teleop");

        waitForStart();

        robot.releaseBack();
        robot.releaseFront();

        telemetry.clearAll();

        SkystoneLocation = robot.detectSkystone();

        telemetry.addData("Position", SkystoneLocation);
        telemetry.update();

        robot.alignSkystone(SkystoneLocation);
        robot.killAll();
        robot.resetEncoders();

        robot.closePipeline();

        if(SkystoneLocation == "Left" || SkystoneLocation == "NOT FOUND"){

            robot.strafeRightWithEnc(0.6, 1832, this);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(0.6, 400, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(85, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeLeftWithEnc(0.9, 4500, this);
            robot.strafeLeftWithEnc(0.9, 6550, this);
            robot.resetEncoders();
            robot.killAll();
            robot.resetEncoders();
            robot.bckWithTime(0.3, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            robot.clampBack();
            sleep(500);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(1, 850, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnRight180(1,this);
            robot.killAll();
            robot.resetEncoders();
            robot.unclampBack();
            sleep(400);
            robot.fwdWithEncoderNoCorrect(0.6, 200, this);
            robot.resetEncoders();
            robot.turnLeftEnc(0.6, 600, this);
            robot.turnLeftGyro(85, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(1, 6340, this);
            robot.killAll();
            robot.reOrient(this);
            robot.killAll();
            robot.resetEncoders();
            robot.setMaintainedHeading(0);
            robot.fwdWithTime(0.3, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 450, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 550, this);
            robot.killAll();
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 550, this);
            robot.turnLeftGyro(85, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeLeftWithEnc(0.9, 4500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.reOrient(this);
            robot.setMaintainedHeading(0);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseFront();
            //sleep(1000);
            robot.killAll();
            robot.resetEncoders();
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(0);
            robot.fwdWithEncoder(1, 1000, this);
            robot.killAll();
            robot.resetEncoders();

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
            robot.strafeLeftWithEnc(0.6, 500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(85, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeLeftWithEnc(0.9, 4500, this);
            robot.strafeLeftWithEnc(0.9, 5600, this);
            robot.resetEncoders();
            robot.killAll();
            robot.resetEncoders();
            robot.bckWithTime(0.3, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseFront();
            robot.clampBack();
            sleep(500);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(1, 850, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnRight180(1,this);
            robot.killAll();
            robot.resetEncoders();
            robot.unclampBack();
            sleep(400);
            robot.fwdWithEncoderNoCorrect(0.6, 200, this);
            robot.resetEncoders();
            robot.turnLeftEnc(0.6, 600, this);
            robot.turnLeftGyro(85, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(1, 6140, this);
            robot.killAll();
            robot.reOrient(this);
            robot.killAll();
            robot.setMaintainedHeading(0);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 450, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 700, this);
            robot.killAll();
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 250, this);
            robot.turnLeftGyro(85, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeLeftWithEnc(0.9, 4500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.reOrient(this);
            robot.setMaintainedHeading(0);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseFront();
            //sleep(1000);
            robot.killAll();
            robot.resetEncoders();
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(0);
            robot.fwdWithEncoder(1, 1000, this);
            robot.killAll();
            robot.resetEncoders();

        }

        if(SkystoneLocation == "Right"){

            robot.strafeRightWithEnc(0.6, 1832, this);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(0.6, 180, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(85, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeLeftWithEnc(0.9, 4500, this);
            robot.strafeLeftWithEnc(0.9, 5500, this);
            robot.resetEncoders();
            robot.killAll();
            robot.resetEncoders();
            robot.bckWithTime(0.3, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            robot.clampBack();
            sleep(500);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(1, 850, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnRight180(1,this);
            robot.killAll();
            robot.resetEncoders();
            robot.unclampBack();
            sleep(400);
            robot.fwdWithEncoderNoCorrect(0.6, 200, this);
            robot.resetEncoders();
            robot.turnLeftEnc(0.6, 600, this);
            robot.turnLeftGyro(85, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(1, 5400, this);
            robot.killAll();
            robot.reOrient(this);
            robot.killAll();
            robot.setMaintainedHeading(0);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 450, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 700, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(85, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeLeftWithEnc(0.9, 4500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.reOrient(this);
            robot.setMaintainedHeading(0);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseFront();
            //sleep(1000);
            robot.killAll();
            robot.resetEncoders();
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(0);
            robot.fwdWithEncoder(1, 1000, this);
            robot.killAll();
            robot.resetEncoders();

        }

    }

}
