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
    public static double kP = 0, kI = 0, kD = 0, kF = 0;
    private double ff;
    private PIDController armController = new PIDController(kP, kI, kD);
    private int armTarget = 0;
    private final double ticks_in_degrees = arm.getCPR()/360;
    private int armPos = 0;
    private double armPower;

    //constructor
    public ArmSubsystem(MotorEx arm, MotorEx slideLeft, Telemetry telemetry) {
        this.arm = arm;
        this.slideLeft = slideLeft;
        this.telemetry = telemetry;
    }

    public void setArm(double targetAngle) {
        ff = kF * Math.cos(Math.toRadians(targetAngle / ticks_in_degrees));
        armPower = armController.calculate(armPos, targetAngle) + ff;
    }

    public void setSlides(double pos, double vel, double accel) {

    }

    @Override
    public void periodic() {
        armPos = slideLeft.getCurrentPosition();
        telemetry.addData("armAngle", armPos/ticks_in_degrees);

    }
}
