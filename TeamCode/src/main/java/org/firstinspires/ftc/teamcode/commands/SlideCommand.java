package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class SlideCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private DoubleSupplier targetInches;

    public SlideCommand(ArmSubsystem armSubsystem, DoubleSupplier targetInches) {
        this.armSubsystem = armSubsystem;
        this.targetInches = targetInches;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute(){
        armSubsystem.setSlide(targetInches.getAsDouble()*36);
    }

}
