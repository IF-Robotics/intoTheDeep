package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armFrontHighChamberX;
import static org.firstinspires.ftc.teamcode.other.Globals.armFrontHighChamberY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchFrontHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollFrontHighChamber;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class HighChamberCommand extends SequentialCommandGroup {

    public HighChamberCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
                //angle the arm to right angle
                new WaitForArmCommand(armSubsystem, 10, 5),
                //extend
                new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, armFrontHighChamberY),
                //spin intake around
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontHighChamber, rollFrontHighChamber)
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }

}
