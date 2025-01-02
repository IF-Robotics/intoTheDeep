package org.firstinspires.ftc.teamcode.subSystems;

import java.util.Optional;
import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    Telemetry telemetry;

    ColorRange blue = new ColorRange(
            ColorSpace.HSV,
            new Scalar( 100, 40, 40),
            new Scalar(140, 255, 255)
    );
    ColorBlobLocatorProcessor locatorProcess = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(blue)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.entireFrame())
            .setBlurSize(5)
            .setDrawContours(true)
            .build();

    VisionPortal visionPortal;

    public VisionSubsystem(CameraName camera, Telemetry telemetry){
        this.telemetry = telemetry;

        ColorBlobLocatorProcessor.BlobFilter areaFilter =
                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 30000, 100000);
        locatorProcess.addFilter(areaFilter);
        ColorBlobLocatorProcessor.BlobFilter ratioFilter =
                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.35, 3);
        locatorProcess.addFilter(ratioFilter);
        ColorBlobLocatorProcessor.BlobSort largestSort =
                new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING);
        locatorProcess.setSort(largestSort);
        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(locatorProcess)
                .build();
    }

    @Override
    public void periodic(){
        telemetry.addData("Sample Skew", getSampleSkew().orElse(-99999.0));
    }

    /**
     * Gets Sample Skew in Degrees
     * @return Optional Double, can return Optional.empty to account for when vision doesn't see anything
     */
    public Optional<Double> getSampleSkew(){
        List<ColorBlobLocatorProcessor.Blob> blobs = locatorProcess.getBlobs();

        if(blobs.size() == 0){return Optional.empty();}

        RotatedRect boxFitBlob = blobs.get(0).getBoxFit();
        //This math is essentially to find the angle considering that the long side is vertical would be 0 degrees
        //This math is likely unecessary but just to be safe added
        Point[] vertices = new Point[4];
        boxFitBlob.points(vertices);

//        for(int i = 0; i < 4; i++){
//            telemetry.addData("Vertex"+i+"X before change", vertices[i].x);
//            telemetry.addData("Vertex"+i+"Y before change", vertices[i].y);
//        }


        //Find the vertex with the max x value
        double max = Double.MIN_VALUE;
        int maxIndex = -1;
        for (int i = 0; i<4; i++){
            if(vertices[i].x > max){
                max = vertices[i].x;
                maxIndex = i;
            }
        }

        //Use this max x value to ensure that we're properly finding the two smallest x values
        int min1 = maxIndex;
        int min2 = maxIndex;
        for (int i=0; i<4; i++) {
            if (vertices[i].x < vertices[min1].x) {
                min2 = min1;
                min1 = i;
            } else if (vertices[i].x < vertices[min2].x) {
                min2 = i;
            }
        }

//        telemetry.addData("min1 index", min1);
//        telemetry.addData("min2 index", min2);

        //Ensure that indicies 0 and 1 are the minimum x values
        Point temp = vertices[0];
        vertices[0] = vertices[min1];
        vertices[min1] = temp;
        temp = vertices[1];
        vertices[1] = vertices[min2];
        vertices[min2] = temp;

//        for(int i = 0; i < 4; i++){
//            telemetry.addData("Vertex"+i+"X before change", vertices[i].x);
//            telemetry.addData("Vertex"+i+"Y before change", vertices[i].y);
//        }

        //Sometimes, if the x values are the same, it won't get organized properly. With this, we ensure that the other values will be organized properly
        //This is cause we want it to be organized as 0 and 1 are the lowest x values, but 3 is always farther to 0 than 2
        //The logic doesn't make sense because it was added as an afterthought
        double side2 = Math.hypot(
                (vertices[0].x - vertices[2].x),
                (vertices[0].y - vertices[2].y)
        );
        double side3 = Math.hypot(
                (vertices[0].x - vertices[3].x),
                (vertices[0].y - vertices[3].y)
        );
        //Ensure that the indices of 2 and 3 are organized by smaller x value
        if (side2 > side3) {
            temp = vertices[2];
            vertices[2] = vertices[3];
            vertices[3] = temp;
            side2 = side3;
        }

//        for(int i = 0; i < 4; i++){
//            telemetry.addData("Vertex"+i+"X after change", vertices[i].x);
//            telemetry.addData("Vertex"+i+"Y after change", vertices[i].y);
//        }

        //Find distances to find longer side of rectangle
        double side1 = Math.hypot(
                (vertices[0].x - vertices[1].x),
                (vertices[0].y - vertices[1].y)
        );

        double angle = 0;

        if(side1 > side2){
            angle = Math.atan((vertices[2].y - vertices[0].y)/(vertices[2].x - vertices[0].x));
        }
        else{
            angle = Math.atan((vertices[1].y - vertices[0].y)/(vertices[1].x - vertices[0].x));
        }

//        telemetry.addData("side1", side1);
//        telemetry.addData("side2", side2);
//        telemetry.addData("side1longer", side1>side2);
        return Optional.of(Math.toDegrees(angle));
    }

}
