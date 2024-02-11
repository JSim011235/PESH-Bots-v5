package org.firstinspires.ftc.teamcode._Vision._RockPipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RockRedPipelineRevamp extends OpenCvPipeline {
    Mat mat = new Mat();

    public enum Location9{
        RIGHT,
        MIDDLE,
        LEFT
    }

    private volatile Location9 location0;
    private String finalLocation;
    static final Rect BMiddle = new Rect(
            new Point(145+80, 160+100),
            new Point(295+120, 60+100));
    static final Rect BRight = new Rect(
            new Point(430+200, 200+200),
            new Point(560+200, 90+100));

    static final double PERCENT_COLOR_THRESHOLD = 0.20;
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0,100,85);
        Scalar highHSV = new Scalar(10,255,255);

        Core.inRange(mat,lowHSV,highHSV,mat);

        Mat middle = mat.submat(BMiddle);
        Mat right = mat.submat(BRight);

        double middleValue = Core.sumElems(middle).val[0] / BMiddle.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / BRight.area() / 255;



        boolean onRight = rightValue >PERCENT_COLOR_THRESHOLD;
        boolean onMiddle = middleValue>PERCENT_COLOR_THRESHOLD;

        if (onMiddle){
            location0 = Location9.MIDDLE;
            finalLocation = "center";
        }
        else if (onRight){
            location0 = Location9.RIGHT;
            finalLocation = "right";

        }
        else{
            location0 = Location9.LEFT;
            finalLocation = "left";

        }
        Scalar False = new Scalar(0,100,85);
        Scalar True = new Scalar(10,255,255);


        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat,BRight , location0 == Location9.RIGHT? True:False);
        Imgproc.rectangle(mat,BMiddle, location0 == Location9.MIDDLE? True :False);

        middle.release();
        right.release();
        return mat;
    }
    public String getLocation(){
        return finalLocation;
    }
}
