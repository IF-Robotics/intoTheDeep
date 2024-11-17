package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

public class WaitForSlideCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double targetExtension;
    private double tolerance;

    public WaitForSlideCommand(ArmSubsystem armSubsystem, double targetExtention, double tolerance) {
        this.armSubsystem = armSubsystem;
        this.targetExtension = targetExtention;
        this.tolerance = tolerance;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute(){
        armSubsystem.setSlide(targetExtension);
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(armSubsystem.getSlideError()) < tolerance){
            return true;
        } else {
            return false;
        }
    }

}
