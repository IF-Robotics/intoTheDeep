package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.other.Globals.armCloseIntakeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armCloseIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketY;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighChamberX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighChamberY;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.outtakePower;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenIntake;

import org.firstinspires.ftc.teamcode.Auto.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoLeft")

public class AutoLeft extends Auto{

    public void run() {

        driveSubsystem.driveToPoint(0, 26, 0);
        armSubsystem.setArmCoordinates(armHighChamberX, armHighChamberY);
        // output intake for high chamber

        driveSubsystem.driveToPoint(0, 20, 0);
        driveSubsystem.driveToPoint(-35, 20, 0);
        armSubsystem.setArmCoordinates(armCloseIntakeX, armCloseIntakeY);
        // intake in
        // Find out if targetheading can be set at the same time as targetx/y
        // coordinate to drop
        driveSubsystem.driveToPoint(-35, 8, 45);
        armSubsystem.setArmCoordinates(armHighBasketX, armHighBasketY);
        // Outtake specimen from intake
        armSubsystem.setArmCoordinates(armCloseIntakeX, armCloseIntakeY);
        driveSubsystem.driveToPoint(-35, 8, 45);
        driveSubsystem.driveToPoint(-45, 20, 0);
        // intake in
        driveSubsystem.driveToPoint(-35, 8, 45);
        armSubsystem.setArmCoordinates(armHighBasketX, armHighBasketY);
        // Outtake specimen from intake
        armSubsystem.setArmCoordinates(armCloseIntakeX, armCloseIntakeY);
        driveSubsystem.driveToPoint(-35, 8, -90);
        driveSubsystem.driveToPoint(-35, 20, -90);



    }
}
