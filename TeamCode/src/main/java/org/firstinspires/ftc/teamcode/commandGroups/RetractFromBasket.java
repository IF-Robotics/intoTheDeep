package org.firstinspires.ftc.teamcode.commandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class RetractFromBasket extends SequentialCommandGroup {

    public RetractFromBasket(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
                
        );
        addRequirements(armSubsystem, intakeSubsystem);
    }
}
