package org.firstinspires.ftc.teamcode.subSystems;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmSubsystem extends SubsystemBase {

    private MotorEx arm, slideL;
    private MotorGroup slide;
    private ServoEx diffyLeft, diffyRight;
    private AnalogInput armEncoder;
    private Telemetry telemetry;


    //arm PIDF
    public static double kParm = 0.07, kIarm = 0, kDarm = 0.01, kFarm = .3;
    public static double armAngleOffset = 113.5;
    private double ff;
    private PIDController armController;
    private double setArmTargetAngle;
    private double armPower;
    private double rawAngle;
    private double correctedAngle;

    //slide pidf
    public static double slideKP = .6, slideKI = 0.0, slideKD = 0.0, slideKF = 0.1;
    private PIDController slideController;
    private final double ticksPerIn = 2786/32.75;
    private int slideTicks = 1;
    private double slidePower;
    private double slideExtention;
    public static double slideWristOffset = 7.75;
    private double setSlideTarget = 8;

    //arm coordinates
    private double slideTargetIn;
    private double armTargetAngle;
    private double armHeight = (24.5 + 24 + 6*24) / 25.4;

    //manualArm
    private double armManualPower;
    private double slideManualPower;

    //constructor
    public ArmSubsystem(MotorEx arm, MotorEx slideL, MotorGroup slide, ServoEx diffyLeft, ServoEx diffyRight, AnalogInput armEncoder, Telemetry telemetry) {
        this.arm = arm;
        this.slide = slide;
        this.slideL = slideL;
        this.armEncoder = armEncoder;
        this.telemetry = telemetry;
    }

    public void manualArm(double armPower, double slidePower){
        armManualPower = -armPower;
        slideManualPower = slidePower;
    }

    public void setArm(double targetAngle) {
        setArmTargetAngle = targetAngle;
    }

    public void setSlide(double targetInches) {
        setSlideTarget = targetInches;
    }

    public void setArmCoordinates(double x, double y){
        slideTargetIn = Math.sqrt(Math.pow(x, 2) + Math.pow(y - armHeight, 2));
        armTargetAngle = Math.toDegrees(Math.atan((y - armHeight)/x));
        setSlide(slideTargetIn);
        if(x < 0) {
            setArm(180 + armTargetAngle);
        } else {
            setArm(armTargetAngle);
        }
    }


    @Override
    public void periodic() {
        slideTicks = slideL.getCurrentPosition();
        rawAngle = armEncoder.getVoltage()/3.3 * 360;;

        if(rawAngle <= 360 && rawAngle > 203){
            correctedAngle = armAngleOffset + (360 - rawAngle);
        } else {
            correctedAngle = armAngleOffset - rawAngle;
        }

        //arm pid
        armController = new PIDController(kParm, kIarm, kDarm);
        ff = kFarm * Math.cos(Math.toRadians(correctedAngle));
        armPower = armController.calculate(correctedAngle, setArmTargetAngle) + ff;
        /*telemetry.addData("ff", ff);
        telemetry.addData("cos", Math.cos(Math.toRadians(angle)));;
        telemetry.addData("targetAngle", targetAngle);
        telemetry.addData("error", targetAngle - angle);*/

        //slide pid
        slideController = new PIDController(slideKP, slideKI, slideKD);
        slidePower = slideController.calculate(slideExtention, setSlideTarget) + slideKF;
        //telemetry.addData("targetIN", targetInches);
        //telemetry.addData("slideTicks", slideTicks);
        telemetry.addData("slideError", setSlideTarget - slideExtention);

        if(manualArm){
            arm.set(armManualPower);
            slide.set(slideManualPower);
        } else {
            arm.set(armPower);
            slide.set(slidePower);
        }


        telemetry.addData("armAngle", correctedAngle);
        telemetry.addData("armPower", armPower);
        telemetry.addData("error", setArmTargetAngle - correctedAngle);

        slideExtention = slideTicks/ticksPerIn + slideWristOffset;
        telemetry.addData("slidePower", slidePower);
        telemetry.addData("slideExtention", slideExtention);

        telemetry.addData("xPos", slideExtention * Math.cos(Math.toRadians(correctedAngle)));
        telemetry.addData("yPos", slideExtention * Math.sin(Math.toRadians(correctedAngle)) + armHeight);

    }

}
