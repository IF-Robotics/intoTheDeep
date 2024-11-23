package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class HighChamberCommand extends SequentialCommandGroup {

    public HighChamberCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
                //extend
                new ArmCoordinatesCommand(armSubsystem, armHighChamberX, armHighChamberY),
                //spin intake around
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchTeleopHighChamber, rollTeleopHighChamber)
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }

}
