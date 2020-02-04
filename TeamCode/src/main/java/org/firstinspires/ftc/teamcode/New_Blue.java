package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.Field_Locations;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.opencv.core.Mat;

@Autonomous
public class New_Blue extends LinearOpMode {

    String SkystoneLocation;
    String grab = "Grab";
    String drop = "Drop";

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap, true);

        Field_Locations locations = new Field_Locations();

        ConstantInterpolator FACING_LZ = new ConstantInterpolator(Math.toRadians(0));

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        SkystoneLocation = drive.detectSkystone();

        drive.setPoseEstimate(new Pose2d(-36, 65, Math.toRadians(0)));

        if(SkystoneLocation == "Left" || SkystoneLocation =="NOT FOUND") {

            drive.backYkA.setPosition(0.6);
            drive.backYk.setPosition(0.5);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.blueSkystone4V, FACING_LZ)
                            .build()
            );

            drive.setPoseEstimate(locations.blueSkystone4P);

            //TODO: add grab code

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.2);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .lineTo(locations.bluePassingVector, FACING_LZ)
                    .lineTo(locations.blueFoundationPlate1V, FACING_LZ)
                    .build()
            );

            //TODO: add drop code

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.5);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.2);

            double offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnTo(Math.toRadians(0));

            drive.setPoseEstimate(locations.blueFoundationPlate1P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .lineTo(locations.bluePassingVector1, FACING_LZ)
                    .lineTo(locations.blueSkystone1V, FACING_LZ)
                    .build()
            );

            drive.turnTo(Math.toRadians(0));

            //TODO: add grab code


            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.2);

            drive.setPoseEstimate(locations.blueSkystone1P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.bluePassingVector2, FACING_LZ)
                            .lineTo(locations.blueFoundationPlate2V, FACING_LZ)
                            .build()
            );

            drive.turnTo(Math.toRadians(0));

            //TODO: add drop code
            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.5);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.2);

            drive.turnTo(Math.toRadians(0));

            drive.setPoseEstimate(locations.blueFoundationPlate2P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.bluePassingVector2, FACING_LZ)
                            .lineTo(locations.blueSkystone2V, FACING_LZ)
                            .build()
            );

            drive.turnTo(Math.toRadians(0));

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.2);

            drive.setPoseEstimate(locations.blueSkystone2P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .lineTo(locations.bluePassingVector2, FACING_LZ)
                    .lineTo(locations.blueFoundationPlate3V, FACING_LZ)
                    .build()
            );

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.5);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.2);

            drive.setPoseEstimate(locations.blueFoundationPlate3P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .lineTo(locations.bluePlateGrabVector, FACING_LZ)
                    .build()
            );

            drive.turnTo(90);

            drive.setPoseEstimate(new Pose2d(50, 28, drive.imu.getAngularOrientation().firstAngle));

            drive.relayPose(telemetry, drive);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .back(5)
                    .build()
            );

            //TODO: clamp code

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .forward(10)
                    .build()
            );

            drive.turnLeftGyro(Math.toRadians(90), this, telemetry);

            boolean positive = true;
            boolean arm = false;
            while (drive.imu.getAngularOrientation().firstAngle != 180 && positive && opModeIsActive()){

                if(drive.imu.getAngularOrientation().firstAngle >= -180 && drive.imu.getAngularOrientation().firstAngle < 0){
                    positive = false;
                }

                //State machine for simultaneous action
                if(drive.imu.getAngularOrientation().firstAngle > 120 && !arm){
                    //TODO: drop front arm
                    arm = true;
                }

                drive.LF.setPower(-0.8);
                drive.LB.setPower(0.8);
                drive.RF.setPower(-0.8);
                drive.RB.setPower(0.8);
            }
            drive.killAll();

            //TODO: unclamp code

            //TODO: drop front wheels

            drive.relayPose(telemetry, drive);

            //TODO: set new pose after empirical analysis

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .lineTo(new Vector2d(36,36), new LinearInterpolator(0, 180))
                    .build()
            );

            //TODO: raise front arm

            //TODO: tape measure code

            sleep(1000);

        }



    }

}
