package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

import java.util.Optional;

public class VisionClawCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private VisionSubsystem visionSubsystem;
    private Optional<Double> angleToSet;

    //(is actually the roll conversion but whateva
    private final double kPitchConversion = 2.33;

    public VisionClawCommand(IntakeSubsystem intakeSubsystem, VisionSubsystem visionSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(intakeSubsystem, visionSubsystem);
    }

    @Override
    public void execute() {
        angleToSet = visionSubsystem.getSampleSkew();
        if (angleToSet.isPresent()){
            intakeSubsystem.setDiffy(angleToSet.get()*kPitchConversion);
        }
    }

}
