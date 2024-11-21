package org.firstinspires.ftc.teamcode.other;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
//intake subsystem
    //intaking
    public static int pitchWhenIntake = 0;
    public static int[] pitchesWhenIntake = {0/*normal*/, 105, 210/*OrthogonalIntake*/, 315};
    public static int rollWhenIntake = -190;
    public static int pitchLastLeftAuto = 60;
    public static int rollLastLeftAuto = -190;
    //intake from the wall
    public static int pitchIntakeWall = 0;
    public static int rollIntakeWall = 170;
    // Right left specimens
    public static int pitchRightLeftSpecimen = 0;
    public static int rollRightLeftSpecimen = 0;

    //claw poses
    public static double clawOpen = .72;
    public static double clawExtraOpen = .9;
    public static double clawClose = .38;

    //scoring
    public static int pitchWhenBasket = 0;
    public static int rollWhenBasket = 150;
    public static double pitchWhenHighChamber = 0;
    public static double rollWhenHighChamber = -20;
    public static double autoPitchFrontHighChamber = 40;
    public static double pitchFrontHighChamber = 300;
    public static double rollFrontHighChamber = 140;
    public static double pitchFrontRightHighChamber = 0;
    public static double rollFrontRightHighChamber = 130;
    public static double pitchPlaceFrontHighRightChamber = 0;
    public static double rollPlaceFrontHighRightChamber = 230;


    //home
    public static int rollWhenArmBack = -150;
    public static int rollWhenArmHome = 200;

//arm subsystem
    //arm home
    public static double armHomeX = 7.5;
    public static double armHomeY = 7;
    //arm fold
    public static double armFoldX = 6.5;
    public static double armFoldY = 4.0;
    //arm back
    public static double armBackX = -3;
    public static double armBackY = 16;
    //arm high basket
    public static double armHighBasketX = 0;
    public static double  armHighBasketY = 48;
    //arm when front high chamber
    public static double armFrontHighChamberX = 27;
    public static double armFrontHighChamberY = 17.5;
    public static double autoArmFrontHighChamberY = 18.8;
    //arm when high chamber
    public static double armHighChamberX = -3;
    public static double armHighChamberY = 18.5;
    public static double armRightHighChamberX = -.02;
    public static double armRightHighChamberY = 22.3;

    //arm when intaking from sub
    public static double armReadySubIntakeX = 23;
    public static double armSubIntakeY = 6.5;
    public static double armReadySubIntakeY = 9.5;
    public static double armInSubIntakeY = 8;
    //arm when close (distance) intake
    public static double armCloseIntakeX = 15;
    public static double armCloseIntakeY = 7.5;
    //arm when intaking form the wall
    public static double armIntakeWallX = -4.7;
    public static double armIntakeWallY = 13.9;

    //manual arm boolean
    public static boolean manualArm = false;
    //arm auto park
    public static double armParkLeftAutoX = 11.01;
    public static double armParkLeftAutoY = 17.58;

    //arm when climbing
    //climbing to first rung
    public static double armPositionToClimbX = 16.9;
    public static double armPositionToClimbY = 24.8;
    //climbing to second rung
    public static double armAngleToSecondRungX = 3.5;
    public static double armAngleToSecondRungY = 14.6;
    public static double armExtendPastSecondRungX = 12;
    public static double armExtendPastSecondRungY = 27.5;
    public static double armMoveToSecondRungX = 13.5;
    public static double armMoveToSecondRungY = 27;
    public static double armPositionRobotToEdgeOfFirstRungX = -8;
    public static double armPositionRobotToEdgeOfFirstRungY = 19;
    //completely retracting when climbing
    public static double armCompleteRetractX = 7.0;
    public static double armCompleteRetractY = 4.0;

//drive to point
    public static double translationKP = 0.015;
    public static double translationKI = 0.0;
    public static double translationKD = 0.001;
    //KR is the constant for the root of the pid
    public static double translationKR = .55;
    public static double headingKP = 0.008;
    public static double headingKI = 0.0;
    public static double headingKD = 0.0002;
    public static double headingKR = .5;
    public static double lateralMutliplier = 1.3;

    public static double testX = 0.0;
    public static double testY = 0.0;
    public static double testHeading = 0;

}
