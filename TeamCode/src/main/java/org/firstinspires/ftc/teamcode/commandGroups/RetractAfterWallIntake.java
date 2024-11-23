package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armBackX;
import static org.firstinspires.ftc.teamcode.other.Globals.armBackY;
import static org.firstinspires.ftc.teamcode.other.Globals.armHomeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHomeY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchFrontHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollFrontHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollIntakeWall;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class RetractAfterWallIntake extends SequentialCommandGroup {

    public RetractAfterWallIntake(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                //grab the specimen
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
                new WaitCommand(300),
                //flip up the intake
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontHighChamber, rollFrontHighChamber),
                //arm to home
                new ArmCoordinatesCommand(armSubsystem, armBackX, armBackY)


        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
