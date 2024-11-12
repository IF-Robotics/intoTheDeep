package org.firstinspires.ftc.teamcode.commandGroups;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class RetractAfterIntake extends SequentialCommandGroup{

    public RetractAfterIntake(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
                //life intake up
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, rollWhenIntake),
                //retract slides
                new ArmCoordinatesCommand(armSubsystem, armHomeX, armHomeY),
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
