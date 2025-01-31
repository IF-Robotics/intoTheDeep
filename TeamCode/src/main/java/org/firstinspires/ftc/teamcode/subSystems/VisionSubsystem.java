package org.firstinspires.ftc.teamcode.subSystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import android.util.Log;
import android.util.Size;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;


import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class VisionSubsystem extends SubsystemBase {

    public enum Alliance {
        RED,
        BLUE
    }

    //0,0 is the upper left corner I believe
    public final static double kCameraWidth=320;
    public final static double kDesiredX = kCameraWidth*0.5;

    public final static double kCameraHeight=240;
    public final static double kDesiredY = kCameraHeight*0.5;

    public static int lowAreaFilter = 600;
    public static int lowAreaFilterYellow = 1200;
    public static int highAreaFilter = 2500;
    public static double lowRatioFilter = 1.3;
    public static double lowRatioFilterYellow = 1.5;
    public static double highRatioFilter = 2.8;
    public static Alliance alliance = Alliance.BLUE;
    Telemetry telemetry;

    public static int exposureMillis = 24;//24, 35

    //    ColorRange blue = new ColorRange(
//            ColorSpace.HSV,
//            new Scalar(100, 125, 80),
//            new Scalar(140, 255, 255)
//    );
    ColorRange blue = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 16,   0, 160),
            new Scalar(255, 127, 255)
    );

//    ColorRange red = new ColorRange(
//            ColorSpace.YCrCb,
//            new Scalar( 32, 176,  80),
//            new Scalar(255, 255, 120)
//    );

    ColorRange red = ColorRange.RED;

    ColorRange yellow = new ColorRange(
            ColorSpace.HSV,
            new Scalar(13, 100, 140),
            new Scalar(50, 255, 255)
    );


    ColorBlobLocatorProcessor.Builder allianceLocatorProcessBuilder = new ColorBlobLocatorProcessor.Builder()
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//            .setRoi(ImageRegion.entireFrame())
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))
            .setBlurSize(1)
            .setErodeSize(4);

    ColorBlobLocatorProcessor allianceLocatorProcess;
    ColorBlobLocatorProcessor yellowLocatorProcess = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(yellow)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.8, 0.8, 0.8, -0.8))
            .setBlurSize(1)
            .setErodeSize(6)
            .setDrawContours(true)
            .build();

    VisionPortal visionPortal;

    public VisionSubsystem(CameraName camera, Telemetry telemetry){
        this.telemetry = telemetry;

        if (alliance == Alliance.BLUE){
            allianceLocatorProcessBuilder.setTargetColorRange(blue);
        }
        else{
            allianceLocatorProcessBuilder.setTargetColorRange(red);
        }

        allianceLocatorProcess = allianceLocatorProcessBuilder.build();

        ColorBlobLocatorProcessor.BlobFilter areaFilter =
                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, lowAreaFilter, highAreaFilter);
        ColorBlobLocatorProcessor.BlobFilter ratioFilter =
                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, lowRatioFilter, highRatioFilter);
        ColorBlobLocatorProcessor.BlobSort largestSort =
                new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING);

        allianceLocatorProcess.addFilter(areaFilter);
        allianceLocatorProcess.addFilter(ratioFilter);
        allianceLocatorProcess.setSort(largestSort);

        ColorBlobLocatorProcessor.BlobFilter areaFilterYellow =
                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, lowAreaFilterYellow, highAreaFilter);
        ColorBlobLocatorProcessor.BlobFilter ratioFilterYellow =
                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, lowRatioFilterYellow, highRatioFilter);

        yellowLocatorProcess.addFilter(areaFilterYellow);
        yellowLocatorProcess.addFilter(ratioFilterYellow);
        yellowLocatorProcess.setSort(largestSort);


        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .setCameraResolution(new Size(320, 240))
                .addProcessor(yellowLocatorProcess)
                .addProcessor(allianceLocatorProcess)
                .build();

        waitForSetCameraSettings(10000, 10000000);
    }


    @Override
    public void periodic(){
//        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
//            WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
//            telemetry.addData("Current White Balance Mode", whiteBalanceControl.getMode());
//            telemetry.addData("White Balance Temp", whiteBalanceControl.getWhiteBalanceTemperature());
//            telemetry.addData("Max temp", whiteBalanceControl.getMaxWhiteBalanceTemperature());
//            telemetry.addData("Min temp", whiteBalanceControl.getMinWhiteBalanceTemperature());
//        }
//        telemetry.addData("Sample Skew", getTotalSkew().orElse(-99999.0));
//        telemetry.addData("Alliance Skew", getAllianceSkew().orElse(-99999.0));
//        telemetry.addData("Yellow Skew", getYellowSkew().orElse(-99999.0));
//
//        Optional<List<Double>> allianceOffsets = getAllianceOffsets();
//        if(allianceOffsets.isPresent()) {
//            telemetry.addData("offset x", allianceOffsets.get().get(0));
//            telemetry.addData("offset y", allianceOffsets.get().get(1));
//        }
//
//        Optional<RotatedRect> allianceRect = getAllianceBoxFit();
//        if(allianceRect.isPresent()){
//            telemetry.addData("alliance x", allianceRect.get().center.x);
//            telemetry.addData("alliance y", allianceRect.get().center.y);
//        }
//
//        ColorBlobLocatorProcessor.BlobFilter areaFilter =
//                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, lowAreaFilter, highAreaFilter);
//        ColorBlobLocatorProcessor.BlobFilter ratioFilter =
//                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, lowRatioFilter, highRatioFilter);
//        for(ColorBlobLocatorProcessor process : new ColorBlobLocatorProcessor[]{allianceLocatorProcess, yellowLocatorProcess}){
//            process.removeAllFilters();
//            process.addFilter(areaFilter);
//            process.addFilter(ratioFilter);
//        }

//        setExposure();
    }

    /**
     *
     * (0,0) is at the upper left corner, so postiive x and y values mean growinig south east
     * @return a list of size 2. index 0 is the pixel count tx and index 1 is the pixel count ty
     */
    public Optional<List<Double>> getAllianceOffsets(){
        Optional<RotatedRect> desiredBoxFit = getAllianceBoxFit();
        if(!desiredBoxFit.isPresent()){
            return Optional.empty();
        }

        return Optional.of(getOffsetFromBoxFit(desiredBoxFit.get()));


    }

    public List<Double> getOffsetFromBoxFit(RotatedRect boxfit){

        double tx = boxfit.center.x-kDesiredX;
        double ty = boxfit.center.y-kDesiredY;

        return Arrays.asList(tx,ty);
    }

    public Optional<RotatedRect> getAllianceBoxFit(){
        List<ColorBlobLocatorProcessor.Blob> blobs = allianceLocatorProcess.getBlobs();
        if(blobs.isEmpty()){return Optional.empty();}
        return getClosestBoxFit(blobs);
    }
    public Optional<RotatedRect> getYellowBoxFit(){
        List<ColorBlobLocatorProcessor.Blob> blobs = yellowLocatorProcess.getBlobs();
        if(blobs.isEmpty()){return Optional.empty();}
        return getClosestBoxFit(blobs);
    }

    /**
     * Gets the closest box fit of both alliances
     */
    public Optional<RotatedRect> getTotalBoxFit(){
        List<ColorBlobLocatorProcessor.Blob> yellowBlobs = yellowLocatorProcess.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> allianceBlobs = allianceLocatorProcess.getBlobs();

        //Warning -- DO NOT DO THIS, this will make yellow blobs also account for alliance specific since java references
//        yellowBlobs.addAll(allianceBlobs);

        List<ColorBlobLocatorProcessor.Blob> totalBlobs = new ArrayList<>();
        totalBlobs.addAll(yellowBlobs);
        totalBlobs.addAll(allianceBlobs);

        if(totalBlobs.isEmpty()){return Optional.empty();}

        return getClosestBoxFit(totalBlobs);
    }

    /**
     * Gets Total Skew in Degrees. This means alliance specific and yellow samples
     * @return Optional Double, can return Optional.empty to account for when vision doesn't see anything
     */
    public Optional<Double> getTotalSkew(){
        Optional<RotatedRect> desiredBoxFit = getTotalBoxFit();
        if (!desiredBoxFit.isPresent()){return Optional.empty();}

        return Optional.of(getAngleFromRotatedRect(desiredBoxFit.get()));
    }

    public Optional<Double> getYellowSkew(){
        Optional<RotatedRect> desiredBoxFit = getYellowBoxFit();
        if (!desiredBoxFit.isPresent()){return Optional.empty();}

        return Optional.of(getAngleFromRotatedRect(desiredBoxFit.get()));
    }

    public Optional<Double> getAllianceSkew(){
        Optional<RotatedRect> desiredBoxFit = getAllianceBoxFit();
        if (!desiredBoxFit.isPresent()){return Optional.empty();}

        return Optional.of(getAngleFromRotatedRect(desiredBoxFit.get()));
    }

    public double getAngleFromRotatedRect(RotatedRect boxFitBlob){
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
        return Math.toDegrees(angle);
    }

    public Optional<RotatedRect> getClosestBoxFit(List<ColorBlobLocatorProcessor.Blob> blobs) {
        if(blobs.isEmpty()){return Optional.empty();}

        double lowestDistance = Math.hypot(blobs.get(0).getBoxFit().center.x-kDesiredX, blobs.get(0).getBoxFit().center.y-kDesiredY);
        int lowestIndex = 0;
        for (int i=1; i<blobs.size(); i++){
            double distance = Math.hypot(blobs.get(i).getBoxFit().center.x-kDesiredX, blobs.get(i).getBoxFit().center.y-kDesiredY);
            if(distance<lowestDistance){
                lowestDistance=distance;
                lowestIndex=i;
            }
        }

        return Optional.of(blobs.get(lowestIndex).getBoxFit());
    }

    public boolean setExposure() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
//        Log.i("camera", "exposure: " + exposureControl.getExposure(TimeUnit.MILLISECONDS));
        return exposureControl.setExposure(exposureMillis, TimeUnit.MILLISECONDS);
    }

    public boolean setWhiteBalance() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        return whiteBalanceControl.setMode(WhiteBalanceControl.Mode.AUTO);
//        Log.i("camera", "white balance: " + whiteBalanceControl.getWhiteBalanceTemperature());
//        return whiteBalanceControl.setWhiteBalanceTemperature(8000);
    }

    public boolean waitForSetCameraSettings(long timeoutMs, int maxAttempts) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        boolean haveSetExposure = false;
        boolean haveSetWhiteBalance = false;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            if (!haveSetExposure && setExposure()) {
                haveSetExposure=true;
            }
            if(!haveSetWhiteBalance && setWhiteBalance()) {
                haveSetWhiteBalance=true;
            }
            if(haveSetExposure&&haveSetWhiteBalance){
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }

        Log.e("camera", "Set exposure failed msAfterStart:" + String.valueOf(msAfterStart) + " attempts:" + String.valueOf(attempts));
        return false;
    }

    public void turnOnStreaming(boolean enabled){
        if(enabled){
            visionPortal.resumeStreaming();
        }
        else{
            visionPortal.stopStreaming();
        }
    }
}
