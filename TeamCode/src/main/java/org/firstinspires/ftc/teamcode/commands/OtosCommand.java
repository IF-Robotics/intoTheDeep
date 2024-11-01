package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subSystems.OdoSubsystem;

public class OtosCommand extends CommandBase{
    private OdoSubsystem odoSubsystem;
    private SparkFunOTOS otos;


    public OtosCommand(SparkFunOTOS otos){
        this.otos = otos;
        addRequirements(odoSubsystem);
    }

    @Override
    public void execute(){
        odoSubsystem.ConfigureOtos();

    }
//    @Override
//    public boolean isFinished() {
//        return true;
//    }
}
