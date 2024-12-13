package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armHighChamberX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighChamberY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchPlaceFrontHighRightChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchTeleopHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollPlaceFrontHighRightChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollTeleopHighChamber;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class ScoreHighChamberCommand extends SequentialCommandGroup {

    public ScoreHighChamberCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
                //score the specimen
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber),
                //retract slides slightly
                new ArmCoordinatesCommand(armSubsystem, armHighChamberX, armHighChamberY -2 ),
                //wait
                new WaitCommand(300),
                //open the claw
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber)
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }

}
