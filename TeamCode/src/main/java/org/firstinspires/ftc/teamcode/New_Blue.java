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

        drive.frontYkA.setPosition(0.75);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        SkystoneLocation = drive.detectSkystone();

        drive.setPoseEstimate(new Pose2d(-36, 65, Math.toRadians(0)));

        telemetry.addData("Location", SkystoneLocation);
        telemetry.update();

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
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .lineTo(locations.bluePassingVector, FACING_LZ)
                    .lineTo(locations.blueFoundationPlate1V, FACING_LZ)
                    .build()
            );

            //TODO: add drop code

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.6);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

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
            drive.frontYkA.setPosition(0.75);
            sleep(800);
            drive.backYkA.setPosition(0.25);

            drive.backs.setPosition(1);

            drive.setPoseEstimate(locations.blueSkystone1P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.bluePassingVector1, FACING_LZ)
                            .lineTo(locations.blueFoundationPlate2V, FACING_LZ)
                            .build()
            );

            drive.turnTo(Math.toRadians(0));

            //TODO: add drop code
            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.6);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.2);

            drive.setPoseEstimate(locations.blueFoundationPlate2P);

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
                    .back(15)
                    .build()
            );

            //TODO: clamp code
            drive.backs.setPosition(0);
            sleep(500);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .forward(22)
                    .build()
            );

            while (drive.imu.getAngularOrientation().firstAngle < Math.toRadians(180) && drive.imu.getAngularOrientation().firstAngle > Math.toRadians(0) && opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                //double pwr = Range.clip(Math.abs(0.011*(drive.imu.getAngularOrientation().firstAngle - Math.toRadians(180))), 0.3, 1);

                if(drive.imu.getAngularOrientation().firstAngle > Math.toRadians(120)){
                    drive.frontYkA.setPosition(0.5);
                }

                drive.LF.setPower(-0.7);
                drive.LB.setPower(0.7);
                drive.RF.setPower(-0.7);
                drive.RB.setPower(0.7);
                telemetry.addData("Heading", drive.imu.getAngularOrientation().firstAngle);
                telemetry.update();
            }
            drive.killAll();


            //TODO: unclamp code

            drive.backs.setPosition(1);

            //TODO: drop front wheels

            drive.leftArm.setPosition(1);
            drive.rightArm.setPosition(0);

            drive.turnTo(180);

            drive.frontYkA.setPosition(0.75);

            drive.setPoseEstimate(new Pose2d(45, 48, Math.toRadians(179)));

            drive.relayPose(telemetry, drive);

            //TODO: set new pose after empirical analysis

            //sleep(500000);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .strafeLeft(0.5)
                    .forward(32)
                    .build()
            );

            //TODO: raise front arm


            //TODO: tape measure code

            //sleep(1000);

        }

        if(SkystoneLocation == "Center"){

            drive.backYkA.setPosition(0.6);
            drive.backYk.setPosition(0.5);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.blueSkystone5V, FACING_LZ)
                            .build()
            );

            drive.setPoseEstimate(locations.blueSkystone5P);

            //TODO: add grab code

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.bluePassingVector, FACING_LZ)
                            .lineTo(locations.blueFoundationPlate1V, FACING_LZ)
                            .build()
            );

            //TODO: add drop code

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.6);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            double offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnTo(Math.toRadians(0));

            drive.setPoseEstimate(locations.blueFoundationPlate1P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.bluePassingVector1, FACING_LZ)
                            .lineTo(locations.blueSkystone2V, FACING_LZ)
                            .build()
            );

            drive.turnTo(Math.toRadians(0));

            //TODO: add grab code


            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYkA.setPosition(0.75);
            sleep(800);
            drive.backYkA.setPosition(0.25);

            drive.backs.setPosition(1);

            drive.setPoseEstimate(locations.blueSkystone2P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.bluePassingVector1, FACING_LZ)
                            .lineTo(locations.blueFoundationPlate2V, FACING_LZ)
                            .build()
            );

            drive.turnTo(Math.toRadians(0));

            //TODO: add drop code
            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.6);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.2);

            drive.setPoseEstimate(locations.blueFoundationPlate2P);

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
                            .back(15)
                            .build()
            );

            //TODO: clamp code
            drive.backs.setPosition(0);
            sleep(500);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(22)
                            .build()
            );

            while (drive.imu.getAngularOrientation().firstAngle < Math.toRadians(180) && drive.imu.getAngularOrientation().firstAngle > Math.toRadians(0) && opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                //double pwr = Range.clip(Math.abs(0.011*(drive.imu.getAngularOrientation().firstAngle - Math.toRadians(180))), 0.3, 1);

                if(drive.imu.getAngularOrientation().firstAngle > Math.toRadians(120)){
                    drive.frontYkA.setPosition(0.5);
                }

                drive.LF.setPower(-0.7);
                drive.LB.setPower(0.7);
                drive.RF.setPower(-0.7);
                drive.RB.setPower(0.7);
                telemetry.addData("Heading", drive.imu.getAngularOrientation().firstAngle);
                telemetry.update();
            }
            drive.killAll();


            //TODO: unclamp code

            drive.backs.setPosition(1);

            //TODO: drop front wheels

            drive.leftArm.setPosition(1);
            drive.rightArm.setPosition(0);

            drive.turnTo(180);

            drive.frontYkA.setPosition(0.75);

            drive.setPoseEstimate(new Pose2d(45, 48, Math.toRadians(179)));

            drive.relayPose(telemetry, drive);

            //TODO: set new pose after empirical analysis

            //sleep(500000);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(2)
                            .forward(32)
                            .build()
            );

            //TODO: raise front arm


            //TODO: tape measure code

            //sleep(1000);

        }

        if(SkystoneLocation == "Right"){

            drive.backYkA.setPosition(0.6);
            drive.backYk.setPosition(0.5);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.blueSkystone6V, FACING_LZ)
                            .build()
            );

            drive.setPoseEstimate(locations.blueSkystone6P);

            //TODO: add grab code

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.bluePassingVector, FACING_LZ)
                            .lineTo(locations.blueFoundationPlate1V, FACING_LZ)
                            .build()
            );

            //TODO: add drop code

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.6);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            double offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnTo(Math.toRadians(0));

            drive.setPoseEstimate(locations.blueFoundationPlate1P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.bluePassingVector1, FACING_LZ)
                            .lineTo(locations.blueSkystone3V, FACING_LZ)
                            .build()
            );

            drive.turnTo(Math.toRadians(0));

            //TODO: add grab code


            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYkA.setPosition(0.75);
            sleep(800);
            drive.backYkA.setPosition(0.25);

            drive.backs.setPosition(1);

            drive.setPoseEstimate(locations.blueSkystone3P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.bluePassingVector1, FACING_LZ)
                            .lineTo(locations.blueFoundationPlate2V, FACING_LZ)
                            .build()
            );

            drive.turnTo(Math.toRadians(0));

            //TODO: add drop code
            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.6);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.2);

            drive.setPoseEstimate(locations.blueFoundationPlate2P);

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
                            .back(15)
                            .build()
            );

            //TODO: clamp code
            drive.backs.setPosition(0);
            sleep(500);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(22)
                            .build()
            );

            while (drive.imu.getAngularOrientation().firstAngle < Math.toRadians(180) && drive.imu.getAngularOrientation().firstAngle > Math.toRadians(0) && opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                //double pwr = Range.clip(Math.abs(0.011*(drive.imu.getAngularOrientation().firstAngle - Math.toRadians(180))), 0.3, 1);

                if(drive.imu.getAngularOrientation().firstAngle > Math.toRadians(120)){
                    drive.frontYkA.setPosition(0.5);
                }

                drive.LF.setPower(-0.7);
                drive.LB.setPower(0.7);
                drive.RF.setPower(-0.7);
                drive.RB.setPower(0.7);
                telemetry.addData("Heading", drive.imu.getAngularOrientation().firstAngle);
                telemetry.update();
            }
            drive.killAll();


            //TODO: unclamp code

            drive.backs.setPosition(1);

            //TODO: drop front wheels

            drive.leftArm.setPosition(1);
            drive.rightArm.setPosition(0);

            drive.turnTo(180);

            drive.frontYkA.setPosition(0.75);

            drive.setPoseEstimate(new Pose2d(45, 48, Math.toRadians(179)));

            drive.relayPose(telemetry, drive);

            //TODO: set new pose after empirical analysis

            //sleep(500000);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(2)
                            .forward(32)
                            .build()
            );

            //TODO: raise front arm


            //TODO: tape measure code

            //sleep(1000);

        }



    }

}
