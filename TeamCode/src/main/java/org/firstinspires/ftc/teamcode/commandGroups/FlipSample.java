package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armSubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenBasket;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.FullRetractSlidesUntilCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class FlipSample extends SequentialCommandGroup {
    public FlipSample(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
                //tilts slides down a tad
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY)),
                //wait
                new WaitCommand(200),
                //grab the sample
                new InstantCommand(() -> intakeSubsystem.closeClaw()),
                //wait
                new WaitCommand(150),
                //retract slides & flip up intake
                new ParallelCommandGroup(
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, 130).withTimeout(100),
                        new WaitForArmCommand(armSubsystem, 35, 10).withTimeout(300)
                ),
//                new WaitForSlideCommand(armSubsystem, 8,5),
                new FullRetractSlidesUntilCommand(armSubsystem, 15),
//                new InstantCommand(() -> intakeSubsystem.openClaw())
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, 0, 200).withTimeout(100)
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
