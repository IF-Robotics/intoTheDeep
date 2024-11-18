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
    public static double armWeakKP = 0.03;
    public static double armAngleOffset = 53.5-30;
    private double ff;
    private PIDController armController;
    private double setArmTargetAngle = 0;
    private double armPower;
    private double rawAngle;
    private double correctedAngle = 0;

    //slide pidf
    public static double slideKP = .6, slideKI = 0.0, slideKD = 0.0, slideKF = 0.1;
    private PIDController slideController;
    private final double ticksPerIn = 2786/32.75;
    private int slideTicks = 1;
    private double slidePower = 0;
    private double slideExtention = 0;
    public static double slideWristOffset = 7.75;
    private double setSlideTarget = 7.75;
    private double slideError = 0;

    //arm coordinates
    private double slideTargetIn;
    private double armTargetAngle;
    private double armHeight = (24.5 + 24 + 6*24) / 25.4;
    private double targetX = armFoldX;
    private double targetY = armFoldY;

    //manualArm
    private double armManualPower;
    private double slideManualPower;

    //intakeFromWall
    private boolean wallActive;

    //constructor
    public ArmSubsystem(MotorEx arm, MotorEx slideL, MotorGroup slide, ServoEx diffyLeft, ServoEx diffyRight, AnalogInput armEncoder, Telemetry telemetry) {
        this.arm = arm;
        this.slide = slide;
        this.slideL = slideL;
        this.armEncoder = armEncoder;
        this.telemetry = telemetry;
        wallActive = false;
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
        targetX = x;
        targetY = y;

        //inverse kinematics
        slideTargetIn = Math.sqrt(Math.pow(x, 2) + Math.pow(y - armHeight, 2));
        armTargetAngle = Math.toDegrees(Math.atan2((y - armHeight), x));

        //write
        setArm(armTargetAngle);
        setSlide(slideTargetIn);
    }

    public void setArmY(double y){
        targetY = y;
        setArmCoordinates(targetX, y);
    }

    public void setArmX(double x){
        targetX = x;
        setArmCoordinates(x, targetY);
    }

    //forward kinematics
    public double getCurrentX(){
        return slideExtention * Math.cos(Math.toRadians(correctedAngle));
    }
    public double getCurrentY(){
        return slideExtention * Math.sin(Math.toRadians(correctedAngle)) + armHeight;
    }

    public double getArmAngle(){
        return correctedAngle;
    }

    public double getSlideExtention(){
        return slideExtention;
    }

    public double getSlideError(){
        return slideError;
    }

    public double getArmTarget(){
        return setArmTargetAngle;
    }

    public double getSlideTarget(){
        return setSlideTarget;
    }

    //return intakeWall state
    public boolean getWallState(){
        return wallActive;
    }
    //toggle wall state
    public void toggleWallState(){
        wallActive = !wallActive;
    }

    @Override
    public void periodic() {
        //read
        slideTicks = slideL.getCurrentPosition();
        rawAngle = armEncoder.getVoltage()/3.3 * 360;;

        if(rawAngle <= 360 && rawAngle > 203){
            correctedAngle = armAngleOffset + (360 - rawAngle);
        } else {
            correctedAngle = armAngleOffset - rawAngle;
        }

        //arm pid
        //lowering the kp when the arm is up
        if(correctedAngle > 80){
            armController = new PIDController(armWeakKP, kIarm, kDarm);
        } else {
            armController = new PIDController(kParm, kIarm, kDarm);
        }
        //feed forward
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
        slideError = setSlideTarget - slideExtention;
        telemetry.addData("slideError", slideError);

        //arm manual
        if(manualArm){
            arm.set(armManualPower);
            slide.set(slideManualPower);
        } else {
            //pid power
            arm.set(armPower);
            slide.set(slidePower);
        }

        //calculate slide extension
        slideExtention = slideTicks/ticksPerIn + slideWristOffset;

        telemetry.addData("armAngle", correctedAngle);
        telemetry.addData("armPower", armPower);
        telemetry.addData("armError", setArmTargetAngle - correctedAngle);
        telemetry.addData("armAngleError", setArmTargetAngle - correctedAngle);

        telemetry.addData("slidePower", slidePower);
        telemetry.addData("slideExtention", slideExtention);

        telemetry.addData("xArmPos", getCurrentX());
        telemetry.addData("yArmPos", getCurrentY());

    }

}
