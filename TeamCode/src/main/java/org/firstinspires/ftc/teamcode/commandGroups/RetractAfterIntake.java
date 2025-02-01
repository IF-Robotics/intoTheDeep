package org.firstinspires.ftc.teamcode.commandGroups;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

public class RetractAfterIntake extends SequentialCommandGroup{

    public RetractAfterIntake(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
                //tilts slides down a tad
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY)),
                //wait
                new WaitCommand(300),
                //grab the sample
                new InstantCommand(() -> intakeSubsystem.closeClaw()),
                //wait
                new WaitCommand(100),
                //retract slides & flip up intake
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, 0),
                new WaitForSlideCommand(armSubsystem, 8, 15),
                //move intake out of the way
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE,  pitchWhenBasket, 0)
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }

    public RetractAfterIntake(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ColorSubsystem colorSubsystem){
        Command dropOppositeAllianceSample = new ConditionalCommand(
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, 0, 0),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, 0),
                ()->colorSubsystem.holdingOppositeColor()
        );
        addCommands(
                //tilts slides down a tad
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY)),
                //wait
                new WaitCommand(300),
                //grab the sample
                new InstantCommand(() -> intakeSubsystem.closeClaw()),
                //wait
                new WaitCommand(100),
                dropOppositeAllianceSample,
                //retract slides & flip up intake
                new WaitForSlideCommand(armSubsystem, 8, 15),
                //move intake out of the way
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE,  pitchWhenBasket, 0)
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }

    //Im overloading it, the boolean has no impacgt, who gaf anymore
    public RetractAfterIntake(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ColorSubsystem colorSubsystem, boolean isAuto){
        Command dropOppositeAllianceSample = new ConditionalCommand(
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, 0, 0)
                        .andThen(new InstantCommand(()->cancelAndReextend(intakeSubsystem, armSubsystem))),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, 0),
                ()->colorSubsystem.holdingOppositeColor()
        );
        addCommands(
                //tilts slides down a tad
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY)),
                //wait
                new WaitCommand(300),
                //grab the sample
                new InstantCommand(() -> intakeSubsystem.closeClaw()),
                //wait
                new WaitCommand(100),
                dropOppositeAllianceSample,
                //retract slides & flip up intake
                new WaitForSlideCommand(armSubsystem, 8, 15),
                //move intake out of the way
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE,  pitchWhenBasket, 0)
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }

    public void cancelAndReextend(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem){
        this.cancel();
        CommandScheduler.getInstance().schedule(new IntakeSub(armSubsystem, intakeSubsystem));
    }

}
