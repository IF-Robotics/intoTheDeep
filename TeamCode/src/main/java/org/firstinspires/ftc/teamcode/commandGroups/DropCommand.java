package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class DropCommand extends SequentialCommandGroup {


    public DropCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new ArmCoordinatesCommand(armSubsystem, armCloseIntakeX, armCloseIntakeY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenIntake, rollWhenIntake),
                new WaitCommand(300),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchWhenIntake, rollWhenIntake)
        );

        addRequirements(intakeSubsystem);
    }
}
