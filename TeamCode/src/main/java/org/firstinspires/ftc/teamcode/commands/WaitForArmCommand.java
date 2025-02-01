package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

public class WaitForArmCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double targetAngle;
    private double tolerance;
    private double powerCap = 0;
    private ElapsedTime timer = new ElapsedTime();
    private int loopCount = 0;

    private boolean returnSlides;

    public WaitForArmCommand(ArmSubsystem armSubsystem, double targetAngle, double tolerance) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;


        addRequirements(armSubsystem);
    }

    public WaitForArmCommand(ArmSubsystem armSubsystem, double targetAngle, double tolerance, boolean returnSlides) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
        this.returnSlides=returnSlides;


        addRequirements(armSubsystem);
    }

    public WaitForArmCommand(ArmSubsystem armSubsystem, double targetAngle, double tolerance, double powerCap) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
        this.powerCap = powerCap;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.setArm(targetAngle);
        if(returnSlides){
            armSubsystem.setSlide(7.75);
        }
        timer.reset();
        loopCount = 0;
        if(powerCap != 0){
            armSubsystem.setArmPowerCap(powerCap);
        }
    }

    @Override
    public void execute(){
        armSubsystem.setArm(targetAngle);
        loopCount++;
    }

    @Override
    public void end(boolean interrupted){
        armSubsystem.setArmPowerCap(1);
    }

    @Override
    public boolean isFinished(){
        if(((armSubsystem.getArmAngle() > targetAngle - tolerance) && (armSubsystem.getArmAngle() < targetAngle + tolerance)) && loopCount >= 2){
            return true;
        } else {
            return false;
        }
    }

}
