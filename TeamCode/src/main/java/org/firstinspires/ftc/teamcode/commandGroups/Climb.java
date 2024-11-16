package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

public class Climb extends SequentialCommandGroup {

    public Climb(ArmSubsystem armSubsystem){
        addCommands(
                //Climb to first rung
                new ArmCoordinatesCommand(armSubsystem, armCompleteRetractX, armCompleteRetractY),
                new WaitCommand(1600),

                //Climb to second rung
                //Rotate arm up just past the second rung
                new ArmCoordinatesCommand(armSubsystem, armAngleToSecondRungX, armAngleToSecondRungY),
                new WaitCommand(1000),
                //Extend the slides until the moving hook is above the second rung
                new ArmCoordinatesCommand(armSubsystem, armExtendPastSecondRungX, armExtendPastSecondRungY),
                new WaitCommand(1000),
                //Move arm to second rung
                new ArmCoordinatesCommand(armSubsystem, armMoveToSecondRungX, armMoveToSecondRungY),
                new WaitCommand(500),
                //Move arm back to rotate the robot down while retracting linear slides until first rung is at the end of the robot ramp
                new ArmCoordinatesCommand(armSubsystem, armPositionRobotToEdgeOfFirstRungX, armPositionRobotToEdgeOfFirstRungY),
                new WaitCommand(1000),
                //Retract linear slides completely while moving arm down until the second rung into the stationary hook
                new ArmCoordinatesCommand(armSubsystem, armCompleteRetractX, armCompleteRetractY),
                new WaitCommand(3000)
        );
        addRequirements(armSubsystem);
    }
}
