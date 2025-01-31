package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armHomeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHomeY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenArmHome;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenBasket;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenIntake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class RetractFromBasketAuto extends SequentialCommandGroup {

    public RetractFromBasketAuto(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
                //outtake
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollWhenBasket),
                //wait
                new WaitCommand(50),
                //move intake out of the way
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchIntakeWall, rollWhenIntake),
                //wait
                new WaitCommand(75),
                //retract slides
                new InstantCommand(() -> armSubsystem.setSlide(8)),
                new WaitCommand(150),
                //move arm down
                new ArmCoordinatesCommand(armSubsystem, armHomeX, armHomeY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, 0, rollWhenArmHome)

        );
        addRequirements(armSubsystem, intakeSubsystem);
    }
}
