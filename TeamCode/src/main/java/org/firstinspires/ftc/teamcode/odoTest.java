package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.Trajectories;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.ExpansionHubEx;

@Autonomous
public class odoTest extends LinearOpMode {

    String SkyStoneLocation;

    public void runOpMode(){

        SampleMecanumDriveREVOptimized robot = new SampleMecanumDriveREVOptimized(hardwareMap, true);

        StandardThreeWheelLocalizer odometery = new StandardThreeWheelLocalizer(hardwareMap);

        robot.setPoseEstimate(new Pose2d(0,0,0));

        double voltage = robot.hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
        double voltage2 = robot.hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);

        Trajectories traj = new Trajectories();

        ConstantInterpolator FACING_LZ = new ConstantInterpolator(Math.toRadians(0));

        //TODO: Add servo init positions

        telemetry.addLine("Ready");
        telemetry.addData("Voltage 1", voltage);
        telemetry.addData("Voltage 2", voltage2);


        while(!isStarted()){

            SkyStoneLocation = robot.detectSkystone();
            telemetry.addData("Skystone Location", SkyStoneLocation);
            telemetry.update();

            if(opModeIsActive()){
                break;
            }

        }

        while(odometery.getWheelPositions().get(2) > -24 && opModeIsActive()){

            telemetry.addData("y pos",odometery.getWheelPositions().get(2));
            telemetry.update();

            if(odometery.getWheelPositions().get(2) > -20){
                robot.strafeRight(1);
            }else{
                robot.strafeRight(0.3);
            }

        }

        while(odometery.getWheelPositions().get(0) < 96 && opModeIsActive()){

            telemetry.addData("y pos",odometery.getWheelPositions().get(2));
            telemetry.update();

            if(odometery.getWheelPositions().get(0) < 92){
                robot.fwd(1);
            }else{
                robot.fwd(0.3);
            }

        }

        robot.killAll();

        robot.relayPose(telemetry, robot);

        sleep(500000);



    }

}
