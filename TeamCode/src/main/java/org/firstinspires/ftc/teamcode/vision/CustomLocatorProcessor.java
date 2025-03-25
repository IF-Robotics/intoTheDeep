package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
//import android.util.Log;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class CustomLocatorProcessor implements VisionProcessor {


    public enum COLOR{
        BLUE,
        RED,
        YELLOW
    }

    //TODO: as of now, does not manage region of interests
    private ImageRegion roiImg = ImageRegion.entireFrame();
    private Rect roi;
    private int frameWidth;
    private int frameHeight;
    private boolean drawContours = false;

    private boolean drawRects = true;

    private volatile COLOR color = COLOR.RED;
    private final Scalar blueLower = new Scalar(100, 105, 80);
    private final Scalar blueUpper = new Scalar(140, 255, 255);
    private final Scalar redLower = new Scalar(32, 176,  80);
    private final Scalar redUpper = new Scalar(255, 255, 132);
    private final Scalar yellowLower = new Scalar(13, 100, 100);
    private final Scalar yellowUpper = new Scalar(50, 255, 255);

    private final Object lockColor = new Object();

    private final int preErodeSize = 3;
    private final int dilateSize = 4;
    private final int postErodeSize = 4;
    private final int edgeDilateSize = 3;

    Mat preErodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(preErodeSize, preErodeSize));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilateSize, dilateSize));
    Mat postErodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(postErodeSize, postErodeSize));
    Mat edgeDilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(edgeDilateSize, edgeDilateSize));


    private volatile ArrayList<RotatedRect> userBlobs = new ArrayList<>();

    public CustomLocatorProcessor(){
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        frameWidth = width;
        frameHeight = height;

        roi = roiImg.asOpenCvRect(width, height);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        ArrayList<RotatedRect> blobs = new ArrayList<>();

//        if(frame.channels()!=3){
//            return blobs;
//        }
        COLOR colorCached = COLOR.YELLOW;
        synchronized (lockColor) {
            colorCached = color;
        }

        Mat hsvImage = new Mat();
        Mat mask = new Mat();


        if (colorCached == COLOR.BLUE) {
            //Convert color for color thresholding
            Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_RGB2HSV);

            //Color Thresholding
            Core.inRange(hsvImage, blueLower, blueUpper, mask);
        }
        else if (colorCached == COLOR.RED) {
            //Convert color for color thresholding
            Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_RGB2YCrCb);

            //Color Thresholding
            Core.inRange(hsvImage, redLower, redUpper, mask);
        }
        else if (colorCached == COLOR.YELLOW) {
            //Convert color for color thresholding
            Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_RGB2HSV);

            //Color Thresholding
            Core.inRange(hsvImage, yellowLower, yellowUpper, mask);
        }
        else{
            return blobs;
        }
        hsvImage.release();

        //Morphological operations
        Imgproc.erode(mask, mask, preErodeElement);
//            Imgproc.dilate(mask, mask, dilateElement);
//            Imgproc.erode(mask, mask, postErodeElement);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, dilateElement);


        //Invert
        Core.bitwise_not(mask, mask);

        //Apply mask to colored image, idek, theres a better solution but this is what ended up working
        Mat copy = new Mat();
        Imgproc.cvtColor(frame, copy, Imgproc.COLOR_RGB2GRAY);
        Mat zeros = Mat.zeros(copy.size(), copy.type());
        Core.bitwise_and(copy, zeros, copy, mask);

        mask.release();
        zeros.release();


        //Edge detection
        if(colorCached == COLOR.YELLOW){
            Imgproc.Canny(copy, copy, 30, 150);
        }
        else {
            Imgproc.Canny(copy, copy, 70, 170);
        }

        Imgproc.dilate(copy, copy, edgeDilateElement);

        //Make border so can detect cutoff samples
        Core.copyMakeBorder(copy, copy, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));

        //Find contours
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(copy, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        copy.release();

        //Find minAreaRects
        for (MatOfPoint contour : contours) {
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f((Point[]) contour.toArray()));
            boolean xinleyang = true;

            if (rect.size.area() < 8000) {
                xinleyang = false;
            }
            if (rect.size.area() > 50000) {
                xinleyang = false;
            }

            if (xinleyang) {
                blobs.add(rect);
                if(drawRects) {
                    Point[] vertices = new Point[4];
                    rect.points(vertices);
                    MatOfPoint points = new MatOfPoint(vertices);
                    Imgproc.polylines(frame, java.util.Collections.singletonList(points), true, new Scalar(0, 255, 0), 3);
                }
            }
        }

        if (drawContours) {
            Imgproc.drawContours(frame, contours, -1, new Scalar(255.0, 255.0, 255.0), 1);
        }

        userBlobs = new ArrayList<>(blobs);
        return blobs;
    }
//
//    public ArrayList<RotatedRect> getRects(Mat frame, Scalar low, Scalar high,){
//        //Convert color for color thresholding
//        Mat hsvImage = new Mat();
//        Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_RGB2HSV);
//
//        //Color Thresholding
//        Mat mask = new Mat();
//        Core.inRange(hsvImage, blueLower, blueUpper, mask);
//
//        //Morphological operations
//        Imgproc.erode(mask, mask, preErodeElement);
//        Imgproc.dilate(mask, mask, dilateElement);
//        Imgproc.erode(mask, mask, postErodeElement);
//
//        //Invert
//        Core.bitwise_not(mask, mask);
//
//        //Apply mask to colored image, idek, theres a better solution but this is what ended up working
//        //TODO: Alternatively, could convert copy to gray beforehand. Would likely save some time
//        Mat copy = frame.clone();
//        Mat zeros = Mat.zeros(frame.size(), frame.type());
//        Core.bitwise_and(copy, zeros, copy, mask);
//
//        //Edge detection
//        Imgproc.cvtColor(copy, copy, Imgproc.COLOR_RGB2GRAY);
//        Imgproc.Canny(copy, copy, 70, 170);
//        Imgproc.dilate(copy, copy, edgeDilateElement);
//
//        //Make border so can detect cutoff samples
//        Core.copyMakeBorder(copy, copy, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));
//
//        //Find contours
//        ArrayList<MatOfPoint> contours = new ArrayList<>();
//        Imgproc.findContours(copy, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        //Find minAreaRects
//        for (MatOfPoint contour : contours) {
//            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f((Point[]) contour.toArray()));
//            boolean xinleyang = true;
//
//            if (rect.size.area() < 8000) {
//                xinleyang = false;
//            }
//            if (rect.size.area() > 50000) {
//                xinleyang = false;
//            }
//
//            if (xinleyang) {
//                blobs.add(rect);
//                if(drawRects) {
//                    Point[] vertices = new Point[4];
//                    rect.points(vertices);
//                    MatOfPoint points = new MatOfPoint(vertices);
//                    Imgproc.polylines(frame, java.util.Collections.singletonList(points), true, new Scalar(0, 255, 0), 3);
//                }
//            }
//        }
//
//        if (drawContours) {
//            Imgproc.drawContours(frame, contours, -1, new Scalar(255.0, 255.0, 255.0), 1);
//        }
//
//    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

//        ArrayList<RotatedRect> blobs = (ArrayList<RotatedRect>) userContext;

//        for(RotatedRect rect: blobs){
//            Point[] vertices = new Point[4];
//            rect.points(vertices);
//            MatOfPoint points = new MatOfPoint(vertices);
//            Imgproc.polylines(frame, java.util.Collections.singletonList(points), true, new Scalar(0, 255 , 0), 3);
//        }
    }


    //----------------------------
    //  EXTRANEOUS STUPID METHODS
    //----------------------------

    public ArrayList<RotatedRect> getBlobs(){return userBlobs;}
    public void setColor(COLOR color){
        synchronized (lockColor) {
            this.color = color;
        }
    }
}
