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

    private Gamepad gamepad1;

    private DcMotorEx BL, BR, FL, FR;

    public TeleDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn) {
        this.driveSubsystem = driveSubsystem;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute(){
        driveSubsystem.teleDrive(1.0, strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
    }

}
