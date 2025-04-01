package org.firstinspires.ftc.teamcode.subSystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;


@Config
public class LimelightSubsystem extends SubsystemBase {



    public enum Alliance {
        RED,
        BLUE

    }

    public static Alliance alliance1 = Alliance.RED;

    private final Limelight3A camera;
    private boolean isDataOld = false;
    private LLResult result;
    private double sampleColor = -1;


//    public static double CAMERA_HEIGHT = ;
//    public static double CAMERA_ANGLE = ;
//    public static double TARGET_HEIGHT = ;
//
//    public static double strafeConversionFactor = ;  //whatever numbers that the actual height and stuff is.
//    public static double cameraStrafeToBot = ;
//
//    public static double sampleToRobotDistance = ;

    Telemetry telemetry;


    public LimelightSubsystem(final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");

        this.telemetry = telemetry;

        if (alliance1 == Alliance.BLUE){
            camera.pipelineSwitch(0);  //pipeline 0 is blue
            sampleColor = 0.0;
        }
        else{
            camera.pipelineSwitch(1);  //pipeline 1 is red
            sampleColor = 1.0;
        }
    } //pipeline 2 is yellow

    public void initializeCamera() {
        camera.setPollRateHz(50);
            camera.start();
    }

    @Override
    public void periodic() {
        camera.updatePythonInputs(new double[] {sampleColor, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        result = camera.getLatestResult(); // call this to get the limelight results
            long staleness = result.getStaleness();
            isDataOld = staleness >= 100; //100 ms
            telemetry.addData("Tx", getTx());
            telemetry.addData("Ty", getTy());
            telemetry.addData("Angle", getAngle());
            telemetry.update();

    }

    public double getTx() {
        return result.getTx();
    } //tx is the x distance from the crosshair (center of the screen)

    public double getTy() {
        return result.getTy();
    } //ty is y distance from the crosshair (center of the screen)

    public Double getAngle() {
        return result.getPythonOutput()[3];
    } //angle of the sample (.getPythonOutput()[i] gets the python snapscript outputs. 2 is center and 4 is area.

}

