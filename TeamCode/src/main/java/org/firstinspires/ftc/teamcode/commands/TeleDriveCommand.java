package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class TeleDriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;

    private DoubleSupplier strafe, forward, turn;
    private double power;
    private boolean arcTanZones;
    private int arcTanAngleRange;

    public TeleDriveCommand(DriveSubsystem driveSubsystem, double power, boolean arcTanZones, int arcTanAngleRange, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn) {
        this.driveSubsystem = driveSubsystem;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        this.power = power;
        this.arcTanZones = arcTanZones;
        this.arcTanAngleRange = arcTanAngleRange;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute(){
        driveSubsystem.teleDrive(power, arcTanZones, arcTanAngleRange, strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
    }

}
