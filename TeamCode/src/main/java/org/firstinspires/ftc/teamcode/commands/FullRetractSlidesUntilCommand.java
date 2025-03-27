package org.firstinspires.ftc.teamcode.commands;

import android.provider.Settings;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.other.Globals;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

import java.util.function.BooleanSupplier;

public class FullRetractSlidesUntilCommand extends CommandBase {
    ArmSubsystem armSubsystem;

    private final double endTarget;
    private final double finalTarget;

    public FullRetractSlidesUntilCommand(ArmSubsystem armSubsystem, double endTarget){
        this.armSubsystem=armSubsystem;
        this.endTarget=endTarget;
        this.finalTarget=8.0;
    }

    @Override
    public void initialize(){
        Globals.manualSlides=true;
    }

    @Override
    public void execute(){
        armSubsystem.manualSlide(-2.0);
    }

    @Override
    public boolean isFinished(){
        return armSubsystem.getSlideExtention()<endTarget || armSubsystem.getSlideExtention()>35;
    }

    @Override
    public void end(boolean interrupted){
        Globals.manualSlides=false;
        armSubsystem.setSlide(finalTarget);
    }

}
