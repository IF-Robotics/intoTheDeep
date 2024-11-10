package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class LocalTestCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private GoBildaPinpointDriver pinpoint;
    private Telemetry telemetry;
    private GamepadEx driver;

    private boolean arcTanZones;
    private int arcTanAngleRange;

    private DoubleSupplier strafe, forward, turn;

    public LocalTestCommand(DriveSubsystem driveSubSystem, GoBildaPinpointDriver pinpoint, Telemetry telemetry, GamepadEx driver, boolean arcTanZones, int arcTanAngleRange, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        this.driveSubsystem = driveSubSystem;
        this.pinpoint = pinpoint;
        this.telemetry = telemetry;
        this.driver = driver;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        this.arcTanZones = arcTanZones;
        this.arcTanAngleRange = arcTanAngleRange;

        addRequirements(driveSubSystem);
    }

    @Override
    public void execute(){
        //drive
        driveSubsystem.teleDrive(driver, arcTanZones, arcTanAngleRange, strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
        //update
        pinpoint.update();
        //telemetry
        Pose2D poseEstimate = pinpoint.getPosition();
        telemetry.addData("X", poseEstimate.getX(DistanceUnit.INCH));
        telemetry.addData("Y", poseEstimate.getY(DistanceUnit.INCH));
        telemetry.addData("normalized heading", poseEstimate.getHeading(AngleUnit.DEGREES));
        telemetry.addData("unnormalized Heading", Math.toDegrees(pinpoint.getHeading()));
        telemetry.addData("Yaw scalar", pinpoint.getYawScalar());
        telemetry.addData("pinpoint hz", pinpoint.getFrequency());
        telemetry.update();
    }
}
