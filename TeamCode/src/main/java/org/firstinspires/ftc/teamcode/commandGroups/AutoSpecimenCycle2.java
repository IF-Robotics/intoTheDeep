package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armHighChamberX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighChamberY;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallX;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallY;
import static org.firstinspires.ftc.teamcode.other.Globals.armRightHighChamberX;
import static org.firstinspires.ftc.teamcode.other.Globals.armRightHighChamberY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchFrontRightHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchPlaceFrontHighRightChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollFrontRightHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollPlaceFrontHighRightChamber;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberRight;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallPickUp;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class AutoSpecimenCycle2 extends SequentialCommandGroup {
    public AutoSpecimenCycle2(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem) {
        addCommands(

                new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),


                // drive to specimen on wall
                new DriveToPointCommand(driveSubsystem, new Pose2d(37, -46, Rotation2d.fromDegrees(180)), 20, 5),
                new DriveToPointCommand(driveSubsystem, new Pose2d(37, -50, Rotation2d.fromDegrees(180)), 20, 5),

                //wait
//                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem, wallPickUp, 1, 3),
                //wait
                new WaitCommand(100),


                // Intake specimen from wall
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
//                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -56, Rotation2d.fromDegrees(180)), 1, 5).withTimeout(200),
                //wait
                new WaitCommand(100),
               // set wrist to ready position for high chamber
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontRightHighChamber, rollFrontRightHighChamber),
                new ArmCoordinatesCommand(armSubsystem, armRightHighChamberX, armRightHighChamberY),



                // Drive to high chamber
                new DriveToPointCommand(driveSubsystem, new Pose2d(9, -32, Rotation2d.fromDegrees(180)),3, 5).withTimeout(500),
                new DriveToPointCommand(driveSubsystem, highChamberRight ,1, 5),
                //wait
//                new WaitCommand(200),
                // Score specimen


                //score the specimen
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber),
                //retract slides slightly
                new ArmCoordinatesCommand(armSubsystem, armHighChamberX, armHighChamberY -2 ),
                //wait
                new WaitCommand(100),
                //open the claw
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber)

        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
