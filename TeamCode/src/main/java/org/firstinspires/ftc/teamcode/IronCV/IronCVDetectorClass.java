package org.firstinspires.ftc.teamcode.IronCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
//import org.openftc.easyopencv.examples.PipelineStageSwitchingExample;

import java.util.ArrayList;
import java.util.List;

public class IronCVDetectorClass {

    private int valMid = -1;
    private int valLeft = -1;
    private int valRight = -1;

    private int[] vals = {valMid, valLeft, valRight};

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private float[] midPos = {3.5f/8f+offsetX, 5f/8f+offsetY};//0 = col, 1 = row
    private float[] leftPos = {1.5f/8f+offsetX, 5f/8f+offsetY};
    private float[] rightPos = {5.5f/8f+offsetX, 5f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;


    OpenCvCamera phoneCam;
    public OpenCvCamera webcam;


    public IronCVDetectorClass() {

    }

    public IronCVDetectorClass(float offsetX, float offsetY) {
        this.offsetX = offsetX;
        this.offsetY = offsetY;
    }

    //   StageSwitchingPipeline detector;
//    public void setOffset(float x, float y) {
//        offsetX = x;
//        offsetY = y;
//    }

    public void camSetup (HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
    }


    public int[] getVals() {

        return vals;
    }

    public void updateVals() {
        vals[0] = valMid;
        vals[1] = valLeft;
        vals[2] = valRight;

    }


    //detection pipeline
    public class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();

        @Override
        public Mat processFrame(Mat input)
        {

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            yCbCrChan2Mat.copyTo(all);//copies mat object

            updateVals(input);

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);



            Imgproc.putText(all,"IRONCV - Developed by Vedant Thorat", new Point(5,30),0,1,new Scalar(255,0,0),2);

            if(valLeft < 200 && valMid > 210 && valRight > 210){
                Imgproc.putText(all,"DETECED - LEFT", new Point(180,440),0,1,new Scalar(255,0,0),2);
            }else if(valLeft > 210 && valMid < 200 && valRight > 210){
                Imgproc.putText(all,"DETECTED - MIDDLE", new Point(180,440),0,1,new Scalar(255,0,0),2);
            }else if(valLeft > 210 && valMid > 210 && valRight < 200){
                Imgproc.putText(all,"DETECTED - RIGHT", new Point(180,440),0,1,new Scalar(255,0,0),2);
            }else{
                Imgproc.putText(all,"NOT DETECTED", new Point(180,440),0,1,new Scalar(255,0,0),2);
            }

            return all;
        }

        public void updateVals(Mat input) {
            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

        }

    }

}
