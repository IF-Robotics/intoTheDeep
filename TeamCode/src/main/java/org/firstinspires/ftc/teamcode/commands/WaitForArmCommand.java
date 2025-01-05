package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

public class WaitForArmCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double targetAngle;
    private double tolerance;
    private ElapsedTime timer = new ElapsedTime();

    public WaitForArmCommand(ArmSubsystem armSubsystem, double targetAngle, double tolerance) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;


        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.setArm(targetAngle);
        timer.reset();
    }

    @Override
    public void execute(){
        armSubsystem.setArm(targetAngle);
    }

    @Override
    public boolean isFinished(){
        if((armSubsystem.getArmAngle() > targetAngle - tolerance && armSubsystem.getArmAngle() < targetAngle + tolerance) && timer.milliseconds() > 50){
            return true;
        } else {
            return false;
        }
    }

}
