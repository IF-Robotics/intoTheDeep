package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class colorSubsystem {
    private final RevColorSensorV3 colorSensor;
    private final Telemetry telemetry;

    public colorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, "Color");
        this.telemetry = telemetry;
    }

    public String detectColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (red > green && red > blue && red > 250) {
            return "Red";
        } else if (blue > red && blue > green && blue > 250) {
            return "Blue";
        } else if (red > 50 && green > 50 && blue < green/3) {
            return "Yellow";
        } else {
            return "Unknown";
        }
    }

    public void updateTelemetry() {
        telemetry.addData("Detected Color", detectColor());
        telemetry.addData("Distance (mm)", colorSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.update();
    }
}