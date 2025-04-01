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
    VisionPortal visionPortal;


//    public static double CAMERA_HEIGHT = 307.0 - 16;
//    public static double CAMERA_ANGLE = -45.0;
//    public static double TARGET_HEIGHT = 19.05;
//
//    public static double strafeConversionFactor = 6.6667;
//    public static double cameraStrafeToBot = -20;
//
//    public static double sampleToRobotDistance = 145;

    Telemetry telemetry;


    public LimelightSubsystem(final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");

        this.telemetry = telemetry;

        if (alliance1 == Alliance.BLUE){
            camera.pipelineSwitch(0);
            sampleColor = 0.0;
        }
        else{
            camera.pipelineSwitch(1);
            sampleColor = 1.0;
        }
    }

    public void initializeCamera() {
        camera.setPollRateHz(50);
            camera.start();
    }

    @Override
    public void periodic() {
        camera.updatePythonInputs(new double[] {sampleColor, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        result = camera.getLatestResult();
            long staleness = result.getStaleness();
            isDataOld = staleness >= 100; //100 ms
            telemetry.addData("Tx", getTx());
            telemetry.addData("Ty", getTy());
            telemetry.addData("Angle", getAngle());
            telemetry.update();

    }

    public double getTx() {
        return result.getTx();
    }

    public double getTy() {
        return result.getTy();
    }

    public Double getAngle() {
        return result.getPythonOutput()[3];
    }

}

