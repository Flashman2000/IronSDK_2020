package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Test_Blue extends LinearOpMode {

    Monmon robot = new Monmon();
    String SkystoneLocation;

    @Override
    public void runOpMode(){

        robot.initAuto(hardwareMap, telemetry);

        waitForStart();

        telemetry.clearAll();

        SkystoneLocation = robot.detectSkystone();

        telemetry.addData("Position", SkystoneLocation);
        telemetry.update();

        if(SkystoneLocation == "Left" || SkystoneLocation == "NOT FOUND"){

            robot.diagRightWithEnc(0.75, 0.5, 1300, this);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 800, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 400, this);
            robot.resetEncoders();
            robot.fwdWithEncoder(0.6, 3200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseFront();
            sleep(500);
            robot.bckWithEncoder(0.6, 4100, this);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 350, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 450, this);
            robot.resetEncoders();
            robot.fwdWithEncoder(0.6, 3800, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(500);
            robot.bckWithEncoder(0.6, 400, this);
            robot.killAll();
            robot.resetEncoders();

        }

        if(SkystoneLocation == "Center"){

            robot.diagRightWithEnc(-0.6, -0.45, 1100, this);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 550, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 550, this);
            robot.resetEncoders();
            robot.fwdWithEncoder(0.6, 5000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseFront();
            sleep(500);
            robot.bckWithEncoder(0.6, 4800, this);
            robot.resetEncoders();
            robot.bckWithEncoder(0.1, 400, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 350, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 450, this);
            robot.resetEncoders();
            robot.fwdWithEncoder(0.6, 2000, this);
            robot.fwdWithEncoder(1, 5300, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseFront();
            sleep(500);
            robot.bckWithEncoder(0.6, 400, this);
            robot.killAll();
            robot.resetEncoders();

        }

        if(SkystoneLocation == "Right"){

            robot.diagRightWithEnc(-0.6, -0.45, 1100, this);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 550, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 550, this);
            robot.resetEncoders();
            robot.fwdWithEncoder(0.6, 5000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(500);
            robot.bckWithEncoder(0.6, 4800, this);
            robot.resetEncoders();
            robot.bckWithEncoder(0.1, 400, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 350, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 450, this);
            robot.resetEncoders();
            robot.fwdWithEncoder(0.6, 2000, this);
            robot.fwdWithEncoder(1, 5300, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(500);
            robot.bckWithEncoder(0.6, 400, this);
            robot.killAll();
            robot.resetEncoders();

        }

    }
}
