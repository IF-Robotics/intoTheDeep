package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSubsystem extends SubsystemBase {
    private final RevColorSensorV3 colorSensor;
    private final RevColorSensorV3 distanceSensor;

//    private final AnalogInput analog0;
//
//    private final AnalogInput analog1;

    private final Telemetry telemetry;

    public enum COLOR{
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    public ColorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, "Color");
        this.distanceSensor = hardwareMap.get(RevColorSensorV3.class, "Distance");
//        this.analog0 = hardwareMap.get(AnalogInput.class, "analog0");
//        this.analog1 = hardwareMap.get(AnalogInput.class, "analog1");
        this.telemetry = telemetry;

    }

    public COLOR getColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (red > green && red > blue && red > 1000) {
            return COLOR.RED;
        } else if (blue > red && blue > green && blue > 900) {
            return COLOR.BLUE;
        } else if (red > 50 && green > 50 && blue < green/4) {
            return COLOR.YELLOW;
        } else {
            return COLOR.UNKNOWN;
        }
    }

    public double getColorDistance(){
        return colorSensor.getDistance(DistanceUnit.INCH);
    }



    public void updateTelemetry() {
        telemetry.addData("Color", getColor());
        telemetry.addData("Distance (mm)", getColorDistance());
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.update();
    }


}