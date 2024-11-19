package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenIntake;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenIntake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class IntakeSub extends SequentialCommandGroup {
    public IntakeSub(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
        //wait for arm to be horizontal
        new WaitForArmCommand(armSubsystem, 5.75, 5),
        //arm & intake to correct pos
        new ArmCoordinatesCommand(armSubsystem, armReadySubIntakeX, armReadySubIntakeY),
        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchWhenIntake, rollWhenIntake)
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
