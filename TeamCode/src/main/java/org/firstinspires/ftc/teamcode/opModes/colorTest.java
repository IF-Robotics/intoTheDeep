package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subSystems.colorSubsystem;

@TeleOp(name = "Test Color Sensor")
public class colorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        colorSubsystem colorSensorSubsystem = new colorSubsystem(hardwareMap, telemetry);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            colorSensorSubsystem.updateTelemetry();
            sleep(100);
        }
    }
}