package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmSubsystem extends SubsystemBase {

    private MotorEx arm, slideLeft;
    private Telemetry telemetry;


    //arm PIDF
    public static double kP = 0.5, kI = 0, kD = 0.01, kF = .3;
    private double ff;
    private PIDController armController;
    private int armTarget = 0;
    private final double ticks_in_degrees = 5264/360;
    private int armPos = 0;
    private double armPower;
    private double angle;
    double lastArmTarget = 0;

    //constructor
    public ArmSubsystem(MotorEx arm, MotorEx slideLeft, Telemetry telemetry) {
        this.arm = arm;
        this.slideLeft = slideLeft;
        this.telemetry = telemetry;
    }

    public void setArm(double targetAngle) {
        armController = new PIDController(kP, kI, kD);
        angle = armPos/ticks_in_degrees;
        ff = kF * Math.cos(Math.toRadians(angle));
        armPower = armController.calculate(angle, targetAngle) + ff;
        telemetry.addData("ff", ff);
        telemetry.addData("cos", Math.cos(Math.toRadians(angle)));;
        telemetry.addData("targetAngle", targetAngle);
        telemetry.addData("error", targetAngle - angle);
    }

    public void setSlides(double pos, double vel, double accel) {

    }

    @Override
    public void periodic() {
        armPos = arm.getCurrentPosition();
        arm.set(armPower);
        //setMotor(arm, armPower);

        angle = armPos/ticks_in_degrees;
        telemetry.addData("armAngle", angle);
        telemetry.addData("armPower", armPower);

    }
    /*public void setMotor(MotorEx motorEx, double power){
        double lastTargetPower = 0;

        if(lastTargetPower <= power + .01 || lastTargetPower >= power - .01){

        } else {
            motorEx.set(power);
            lastTargetPower = power;
        }
    }*/
}
