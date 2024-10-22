package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmSubsystem extends SubsystemBase {

    private MotorEx arm, slideL;
    private MotorGroup slide;
    private Telemetry telemetry;


    //arm PIDF
    public static double kParm = 0.5, kIarm = 0, kDarm = 0.01, kFarm = .3;
    private double ff;
    private PIDController armController;
    private final double ticks_in_degrees = 5264/360;
    private int armPos = 0;
    private double armPower;
    private double angle;

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
    public ArmSubsystem(MotorEx arm, MotorEx slideL, MotorGroup slide, Telemetry telemetry) {
        this.arm = arm;
        this.slide = slide;
        this.slideL = slideL;
        this.telemetry = telemetry;
    }

    public void setArm(double targetAngle) {
        armController = new PIDController(kParm, kIarm, kDarm);
        ff = kFarm * Math.cos(Math.toRadians(angle));
        armPower = armController.calculate(angle, targetAngle) + ff;
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
        setSlide(slideTargetIn - 9.2);
        if(x < 0) {
            setArm(180 + armTargetAngle);
        } else {
            setArm(armTargetAngle);
        }
    }


    @Override
    public void periodic() {
        armPos = arm.getCurrentPosition();
        slideTicks = slideL.getCurrentPosition();
        //setMotor(arm, armPower);

        angle = armPos/ticks_in_degrees - 47;
        telemetry.addData("armAngle", angle);
        telemetry.addData("armPower", armPower);

        slideExtention = slideTicks/ticksPerIn;
        telemetry.addData("slideExtention", slideExtention);

    }

}
