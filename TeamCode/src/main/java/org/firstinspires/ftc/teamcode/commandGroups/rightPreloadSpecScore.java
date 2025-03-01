package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armFrontHighChamberX;
import static org.firstinspires.ftc.teamcode.other.Globals.autoArmFrontHighChamberY;
import static org.firstinspires.ftc.teamcode.other.Globals.autoPitchFrontHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollFrontHighChamber;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.firstHighChamberRight;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class rightPreloadSpecScore extends SequentialCommandGroup {

    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ArmSubsystem armSubsystem;

    public rightPreloadSpecScore(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;


        addCommands(
                //raise intake and arm
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, autoPitchFrontHighChamber, rollFrontHighChamber),
                new InstantCommand(() -> armSubsystem.setArm(22)),
                //wait
                new WaitCommand(200),
                //extend slides
                new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, autoArmFrontHighChamberY),
                //wait
                new WaitCommand(400),


                // Drive to high chamber
                // Score specimen
                new DriveToPointCommand(driveSubsystem, firstHighChamberRight,5, 10).withTimeout(1500),
                //open
                
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, autoPitchFrontHighChamber, rollFrontHighChamber),
                new WaitCommand(100),
                //arm to home pos
                new InstantCommand(() -> armSubsystem.setArm(45)),
                new WaitCommand(100),
                new InstantCommand(() -> armSubsystem.setSlide(18))
        );

        addRequirements(intakeSubsystem, armSubsystem);
    }


}
