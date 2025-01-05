package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

public class WaitForSlideCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double targetExtension;
    private double tolerance;
    private ElapsedTime timer = new ElapsedTime();

    public WaitForSlideCommand(ArmSubsystem armSubsystem, double targetExtention, double tolerance) {
        this.armSubsystem = armSubsystem;
        this.targetExtension = targetExtention;
        this.tolerance = tolerance;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute(){
        armSubsystem.setSlide(targetExtension);
    }

    @Override
    public boolean isFinished(){
        if(((Math.abs(armSubsystem.getSlideError()) < tolerance) && armSubsystem.getSlideVelocity() < 10) && timer.milliseconds() > 50){
            return true;
        } else {
            return false;
        }
    }

}
