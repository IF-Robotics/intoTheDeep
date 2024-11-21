package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armCloseIntakeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armCloseIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenIntake;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenIntake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class IntakeCloseCommand extends SequentialCommandGroup {

    public IntakeCloseCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
                new ArmCoordinatesCommand(armSubsystem, armCloseIntakeX, armCloseIntakeY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchWhenIntake, rollWhenIntake),
                new InstantCommand(() -> intakeSubsystem.resetRotateIntake())
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
