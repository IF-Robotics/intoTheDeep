package org.firstinspires.ftc.teamcode.subSystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

public class ColorSubsystem extends SubsystemBase {
    private final RevColorSensorV3 colorSensor;
    private final I2cDeviceSynchSimple i2c;

//    private final AnalogInput analog0;
//
//    private final AnalogInput analog1;

    private final Telemetry telemetry;

    private static final byte LED_BRIGHTNESS = 0x46;

    public enum COLOR{
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    public ColorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, "Color");
        this.i2c=colorSensor.getDeviceClient();
        this.i2c.enableWriteCoalescing(true);
//        this.analog0 = hardwareMap.get(AnalogInput.class, "analog0");
//        this.analog1 = hardwareMap.get(AnalogInput.class, "analog1");
        this.telemetry = telemetry;

        turnOnLED(true);
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
        return colorSensor.getDistance(DistanceUnit.MM);
    }



    public void updateTelemetry() {
        telemetry.addData("Color", getColor());
        telemetry.addData("Distance (mm)", getColorDistance());
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.update();
    }


    public boolean holdingOppositeColor(){
        boolean sampleInBot = getColorDistance()<100;
        boolean oppositeColor = false;
        COLOR bruh = getColor();
        if(VisionSubsystem.alliance == VisionSubsystem.Alliance.BLUE){
//            if(getColor()==COLOR.RED){
            if(bruh== COLOR.BLUE){ //TEST
                oppositeColor=true;
                Log.i("ColorSensorOpposite", "Yes");
            }
        }
        else{
//            if(getColor()== COLOR.BLUE){
            if(bruh== COLOR.RED){ //TEST
                oppositeColor=true;
                Log.i("ColorSensorOpposite", "Yes");
            }
        }

        Log.i("ColorSensorColor", String.valueOf(bruh));
        Log.i("ColorSensorSampleInBot", "Yes");
        Log.i("Red", String.valueOf(colorSensor.red()));
        Log.i("Green", String.valueOf(colorSensor.green()));
        Log.i("Blue", String.valueOf(colorSensor.blue()));

        return sampleInBot && oppositeColor;
    }

    //https://docs.brushlandlabs.com/sensors/color-rangefinder/configuration
    private void setLedBrightness(int value) {
        i2c.write8(LED_BRIGHTNESS, value);
    }

    public void turnOnLED(boolean enable){
        if(enable){
            setLedBrightness(50);
        }
        else{
            setLedBrightness(0);
        }
    }

}