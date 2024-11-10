package org.firstinspires.ftc.teamcode.opModes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.LocalTestCommand;

@TeleOp
public class BetterLocalizationTest extends Robot{

    LocalTestCommand localTestCommand;

    @Override
    public void initialize() {
        super.initialize();
        localTestCommand = new LocalTestCommand(driveSubsystem, pinpoint, telemetry, m_driver, true, 10, m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX);
        schedule(localTestCommand);
    }


}
