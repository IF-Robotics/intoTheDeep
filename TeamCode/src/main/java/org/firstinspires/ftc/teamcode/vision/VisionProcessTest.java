package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class VisionProcessTest implements VisionProcessor {

    ColorRange colorRange;

    //preerode
    int predilateSize = 3;

    //post erode
    int erodeSize = 4;

    //dilate
    int postDilateSize = 4;

    Mat predilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(predilateSize, predilateSize));
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erodeSize, erodeSize));
    Mat postdilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(postDilateSize, postDilateSize));

    public VisionProcessTest(){
        colorRange = new ColorRange(
                ColorSpace.YCrCb,
                new Scalar( 16,   0, 160),
                new Scalar(255, 127, 255)
        );
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Not useful in this case, but we do need to implement it either way
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

//        Mat mask1 = new Mat();
//        Imgproc.cvtColor(frame, mask1, Imgproc.COLOR_RGB2HSV);
//        Imgproc.Canny(mask1, mask1, 20, 120);
//        Imgproc.dilate(mask1, mask1, predilateElement);
//        Imgproc.erode(mask1, mask1, erodeElement);
        // Convert image to HSV color space
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_RGB2HSV);
//

//        //BLUE
//        Scalar lowerBound = new Scalar(100, 105, 80);  // Adjust for target color
//        Scalar upperBound = new Scalar(140, 255, 255);

//        //YELLOW
        Scalar lowerBound = new Scalar(13, 100, 100);  // Adjust for target color
        Scalar upperBound = new Scalar(50, 255, 255);
//
//        // Apply inRange to get the mask
        Mat mask = new Mat();
        Core.inRange(hsvImage, lowerBound, upperBound, mask);

        Imgproc.erode(mask, mask, predilateElement);
        Imgproc.dilate(mask, mask, postdilateElement);
        Imgproc.erode(mask, mask, erodeElement);

        Core.bitwise_not(mask, mask);

        Mat copy = frame.clone();
        Mat bruh = Mat.zeros(frame.size(), frame.type());

        Mat thresholded = new Mat();
        Core.bitwise_and(copy, bruh, copy, mask);
//        Imgproc.cvtColor(mask, frame, Imgproc.COLOR_GRAY2RGB);

//        Core.bitwise_and(frame, mask, frame);
//        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
//
//        // Apply the mask to the original image (bitwise AND to keep colors)
//        Mat filteredImage = new Mat();
//        Core.bitwise_and(frame, frame, frame, mask);

//        Mat bruh = frame.clone();
//
//        Mat mask2 = new Mat();
//        Imgproc.cvtColor(frame, mask2, Imgproc.COLOR_RGB2YCrCb);
//
//        Mat mask = new Mat();
//        Core.inRange(mask2, new Scalar( 32, 176,  80), new Scalar(255, 255, 132), mask2);
////        frame.convertTo(mask, mask.type());/
//        frame.copyTo(frame, mask);

//        Mat mask = new Mat();
//        Mat mask2 = new Mat();

        Imgproc.cvtColor(copy, copy, Imgproc.COLOR_RGB2GRAY);

        //BLUE
//        Imgproc.Canny(copy, copy, 70, 170);

        Imgproc.Canny(copy, copy, 30, 150);

        Imgproc.dilate(copy, copy, predilateElement);


//        Imgproc.cvtColor(bruh, bruh, Imgproc.COLOR_RGB2HSV);
//
//
//        Mat thresholdMask = new Mat();
//
//        Core.inRange(bruh, new Scalar(13, 125, 60), new Scalar(50, 255, 255), thresholdMask);
//
//
//        Core.bitwise_and(bruh, bruh, bruh, thresholdMask);
//        frame.copyTo(frame, mask);


//        Imgproc.cvtColor(mask, frame, Imgproc.COLOR_RGB2GRAY);
//        Imgproc.Canny(mask2, mask2, 100, 250);
//        Core.bitwise_and(mask1, mask2, mask1);
        Core.copyMakeBorder(copy, copy, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255,255,255));
////








        ArrayList<MatOfPoint> contours = new ArrayList<>();
//
        Imgproc.findContours(copy, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);







////
//        for(MatOfPoint contour: contours){
//            MatOfPoint2f ruh = new MatOfPoint2f();
//            Imgproc.approxPolyDP(new MatOfPoint2f((Point[]) contour.toArray()), ruh, 7, true);
//            MatOfPoint poly = new MatOfPoint();
//            ruh.convertTo(poly, CvType.CV_32S);
//
//            // Draw the approximated polygon
//            Imgproc.polylines(frame, java.util.Collections.singletonList(poly), true, new Scalar(0, 255, 0), 1);
//        }








        Imgproc.drawContours(frame,contours, -1, new Scalar(255.0, 255.0, 255.0), 1);

        ArrayList<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();

        for(MatOfPoint contour: contours){
            RotatedRect john = Imgproc.minAreaRect(new MatOfPoint2f((Point[]) contour.toArray()));
            boolean xinleyang = true;

            if(john.size.area()<8000){
                xinleyang=false;
            }
            if(john.size.area()>50000){
                xinleyang=false;
            }

            if(xinleyang) {
                Point[] vertices = new Point[4];
                john.points(vertices);
                MatOfPoint points = new MatOfPoint(vertices);
                Imgproc.polylines(frame, java.util.Collections.singletonList(points), true, new Scalar(0, 255 , 0), 3);
            }
            blobs.add(new BlobImpl(contour));
        }









//        blobs.removeIf(blob -> blob.getContourArea() < 5000);
//        blobs.removeIf(blob-> blob.getBoxFit().size.area()<3000);
//        blobs.removeIf(blob-> blob.getBoxFit().size.area()>15000);
//
//        blobs.removeIf(blob->blob.getAspectRatio()>2.5);



//        Imgproc.dilate(frame,frame, predilateElement);
//        Imgproc.erode(frame, frame, erodeElement);
//        Imgproc.dilate(frame,frame, postdilateElement);

        //WATERSHED ALGO
//        Mat gray = new Mat();

//        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2GRAY);
//        Imgproc.threshold(frame, frame, 0, 255, Imgproc.THRESH_BINARY+Imgproc.THRESH_OTSU);
//
////            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
////            Core.inRange(frame, new Scalar(13, 125, 60), new Scalar(50, 255, 255), frame);
//            Imgproc.erode(frame, frame, postdilateElement);
//            Imgproc.dilate(frame, frame, predilateElement);
//            Imgproc.erode(frame, frame, erodeElement);
////            frame.convertTo(frame, CvType.CV_8UC1);
//            Imgproc.distanceTransform(frame, frame, Imgproc.DIST_L1, 3);
//            Core.normalize(frame, frame, 0.0, 1.0, Core.NORM_MINMAX);

        // Convert to grayscale
//        Mat gray = new Mat();
//        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);
//
//        // Apply threshold
//        Mat binary = new Mat();
//        Imgproc.threshold(gray, binary, 0, 255, Imgproc.THRESH_BINARY_INV + Imgproc.THRESH_OTSU);

//        // Remove noise with morphological operations
//        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
//        Imgproc.morphologyEx(binary, frame, Imgproc.MORPH_OPEN, kernel);
//
////        // Compute distance transform
//        Mat distTransform = new Mat();
//        Imgproc.distanceTransform(binary, frame, Imgproc.DIST_L2, 5);
//        Core.normalize(distTransform, distTransform, 0, 1, Core.NORM_MINMAX);
//
//        // Threshold the distance transform to get markers
//        Mat markers = new Mat();
//        Imgproc.threshold(distTransform, frame, 0.7, 1.0, Imgproc.THRESH_BINARY);
//
//        // Convert markers to 8-bit integer type
//        markers.convertTo(markers, CvType.CV_8U);
//
//        // Apply watershed
//        Imgproc.watershed(frame, markers);
//
//        // Mark boundaries in red
//        for (int i = 0; i < markers.rows(); i++) {
//            for (int j = 0; j < markers.cols(); j++) {
//                if (markers.get(i, j)[0] == -1) {
//                    frame.put(i, j, new double[]{0, 0, 255});
//                }
//            }
//        }
//        return null; // No context objectd
//        return contours;
        return blobs;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either

        ArrayList<ColorBlobLocatorProcessor.Blob> blobs = (ArrayList<ColorBlobLocatorProcessor.Blob>) userContext;

        Paint boundingRectPaint = new Paint();
        boundingRectPaint.setAntiAlias(true);
        boundingRectPaint.setStrokeCap(Paint.Cap.BUTT);
        boundingRectPaint.setStrokeWidth(1);
        boundingRectPaint.setColor(Color.rgb(0, 255, 0));
        for (ColorBlobLocatorProcessor.Blob blob : blobs)
        {
            Point[] rotRectPts = new Point[4];
            blob.getBoxFit().points(rotRectPts);

            for(int i = 0; i < 4; ++i)
            {
//                canvas.drawLine(
//                        (float) (rotRectPts[i].x)*scaleBmpPxToCanvasPx, (float) (rotRectPts[i].y)*scaleBmpPxToCanvasPx,
//                        (float) (rotRectPts[(i+1)%4].x)*scaleBmpPxToCanvasPx, (float) (rotRectPts[(i+1)%4].y)*scaleBmpPxToCanvasPx,
//                        boundingRectPaint
//                );
            }
        }
    }

    class BlobImpl extends ColorBlobLocatorProcessor.Blob
    {
        private MatOfPoint contour;
        private Point[] contourPts;
        private int area = -1;
        private double density = -1;
        private double aspectRatio = -1;
        private RotatedRect rect;

        BlobImpl(MatOfPoint contour)
        {
            this.contour = contour;
        }

        @Override
        public MatOfPoint getContour()
        {
            return contour;
        }

        @Override
        public Point[] getContourPoints()
        {
            if (contourPts == null)
            {
                contourPts = contour.toArray();
            }

            return contourPts;
        }

        @Override
        public int getContourArea()
        {
            if (area < 0)
            {
                area = Math.max(1, (int) Imgproc.contourArea(contour));  //  Fix zero area issue
            }

            return area;
        }

        @Override
        public double getDensity()
        {
            Point[] contourPts = getContourPoints();

            if (density < 0)
            {
                // Compute the convex hull of the contour
                MatOfInt hullMatOfInt = new MatOfInt();
                Imgproc.convexHull(contour, hullMatOfInt);

                // The convex hull calculation tells us the INDEX of the points which
                // which were passed in eariler which form the convex hull. That's all
                // well and good, but now we need filter out that original list to find
                // the actual POINTS which form the convex hull
                Point[] hullPoints = new Point[hullMatOfInt.rows()];
                List<Integer> hullContourIdxList = hullMatOfInt.toList();

                for (int i = 0; i < hullContourIdxList.size(); i++)
                {
                    hullPoints[i] = contourPts[hullContourIdxList.get(i)];
                }

                double hullArea = Math.max(1.0,Imgproc.contourArea(new MatOfPoint(hullPoints)));  //  Fix zero area issue

                density = getContourArea() / hullArea;
            }
            return density;
        }

        @Override
        public double getAspectRatio()
        {
            if (aspectRatio < 0)
            {
                RotatedRect r = getBoxFit();

                double longSize  = Math.max(1, Math.max(r.size.width, r.size.height));
                double shortSize = Math.max(1, Math.min(r.size.width, r.size.height));

                aspectRatio = longSize / shortSize;
            }

            return aspectRatio;
        }

        @Override
        public RotatedRect getBoxFit()
        {
            if (rect == null)
            {
                rect = Imgproc.minAreaRect(new MatOfPoint2f(getContourPoints()));
            }
            return rect;
        }
    }
}
