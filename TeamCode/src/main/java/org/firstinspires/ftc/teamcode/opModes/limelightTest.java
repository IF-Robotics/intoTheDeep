package org.firstinspires.ftc.teamcode.opModes;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subSystems.LimelightSubsystem;
@Config
 @TeleOp(name = "LimelightTest")
public class limelightTest extends LinearOpMode {
    private Limelight3A camera;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        LimelightSubsystem vision = new LimelightSubsystem(hardwareMap, telemetry);
        vision.initializeCamera();
        waitForStart();
        camera.start(); //call waitForStart() and then call camera.start() afterwards to start the camera

        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();
            LimelightSubsystem.alliance1= LimelightSubsystem.Alliance.BLUE;
        }

        CommandScheduler.getInstance().reset();
    }
}