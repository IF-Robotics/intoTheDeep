package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.AnalogInput;

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
    public static double slideKP = 1, slideKI = 0.0, slideKD = 0.0, slideKF = 0.1;
    private PIDController slideController;
    private final double ticksPerIn = 2786/32.75;
    private int slideTicks = 1;
    private double slidePower;
    private double slideExtention;

    //arm coordinates
    private double slideTargetIn;
    private double armTargetAngle;
    private double armHeight = (24.5 + 24 + 6*24) / 25.4;

    //constructor
    public ArmSubsystem(MotorEx arm, MotorEx slideL, MotorGroup slide, ServoEx diffyLeft, ServoEx diffyRight, AnalogInput armEncoder, Telemetry telemetry) {
        this.arm = arm;
        this.slide = slide;
        this.slideL = slideL;
        this.armEncoder = armEncoder;
        this.telemetry = telemetry;
    }

    public void setArm(double targetAngle) {
        setArmTargetAngle = targetAngle;
        armController = new PIDController(kParm, kIarm, kDarm);
        ff = kFarm * Math.cos(Math.toRadians(correctedAngle));
        armPower = armController.calculate(correctedAngle, targetAngle) + ff;
        arm.set(armPower);
        /*telemetry.addData("ff", ff);
        telemetry.addData("cos", Math.cos(Math.toRadians(angle)));;
        telemetry.addData("targetAngle", targetAngle);
        telemetry.addData("error", targetAngle - angle);*/
    }

    public void setSlide(double targetInches) {
        slideController = new PIDController(slideKP, slideKI, slideKD);
        slidePower = slideController.calculate(slideExtention, targetInches) + slideKF;
        slide.set(slidePower);
        //telemetry.addData("targetIN", targetInches);
        //telemetry.addData("slideTicks", slideTicks);
        telemetry.addData("slideError", targetInches - slideExtention);
    }

    public void setArmCoordinates(double x, double y){
        slideTargetIn = Math.sqrt(Math.pow(x, 2) + Math.pow(y - armHeight, 2));
        armTargetAngle = Math.toDegrees(Math.atan((y - armHeight)/x));
        setSlide(slideTargetIn - 7.75);
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
        //setMotor(arm, armPower);

        if(rawAngle <= 360 && rawAngle > 203){
            correctedAngle = armAngleOffset + (360 - rawAngle);
        } else {
            correctedAngle = armAngleOffset - rawAngle;
        }
        telemetry.addData("armAngle", correctedAngle);
        telemetry.addData("armPower", armPower);
        telemetry.addData("error", setArmTargetAngle - correctedAngle);

        slideExtention = slideTicks/ticksPerIn;
        telemetry.addData("slideExtention", slideExtention);

    }

}
