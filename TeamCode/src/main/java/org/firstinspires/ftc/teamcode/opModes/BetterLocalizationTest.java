package org.firstinspires.ftc.teamcode.opModes;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.LocalTestCommand;
import org.firstinspires.ftc.teamcode.commands.SetStartingPosCommand;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

@TeleOp
public class BetterLocalizationTest extends Robot{

    LocalTestCommand localTestCommand;
    SetStartingPosCommand setStartingPosCommand;

    @Override
    public void initialize() {
        super.initialize();

        schedule( new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosRight)));
        localTestCommand = new LocalTestCommand(driveSubsystem, pinpoint, telemetry, m_driver, true, 10, m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX);
        driveSubsystem.setDefaultCommand(localTestCommand);
    }


}
