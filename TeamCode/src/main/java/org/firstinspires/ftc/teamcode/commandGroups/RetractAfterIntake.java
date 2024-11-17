package org.firstinspires.ftc.teamcode.commandGroups;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class RetractAfterIntake extends SequentialCommandGroup{

    public RetractAfterIntake(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
                //tilts slides down a tad
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY)),
                //wait
                new WaitCommand(500),
                //grab the sample
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, rollWhenIntake),
                //wait
                new WaitCommand(200),
                //retract slides & flip up intake
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, rollWhenArmHome),
                new WaitForSlideCommand(armSubsystem, 8, 1),
                //wait
                new WaitCommand(1000),
                //move arm back
                new ArmCoordinatesCommand(armSubsystem, armBackX, armBackY),
                //move intake out of the way
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE,  pitchWhenBasket, rollWhenArmBack)
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
