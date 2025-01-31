package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subSystems.ColorSubsystem;

@TeleOp(name = "Test Color Sensor")
public class colorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ColorSubsystem colorSubsystem = new ColorSubsystem(hardwareMap, telemetry);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            colorSubsystem.updateTelemetry();
            sleep(100);
        }
    }
}