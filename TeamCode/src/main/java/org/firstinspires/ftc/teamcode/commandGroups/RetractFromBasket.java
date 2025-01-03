package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingPosRight;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class RetractFromBasket extends SequentialCommandGroup {

    public RetractFromBasket(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
                //outtake
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, 350, rollWhenBasket),
                //wait
                new WaitCommand(200),
                //move intake out of the way
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, 350, rollWhenIntake),
                //retract slides
                new InstantCommand(() -> armSubsystem.setSlide(8)),
                //wait
                new WaitCommand(500),
                //move arm down
                new ArmCoordinatesCommand(armSubsystem, armHomeX, armHomeY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, 0, rollWhenArmHome)

        );
        addRequirements(armSubsystem, intakeSubsystem);
    }
}
