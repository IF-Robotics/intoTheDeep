package org.firstinspires.ftc.teamcode.other;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
//intake subsystem
    //intaking
    public static int pitchWhenIntake = 0;
    public static int rollWhenIntake = -190;
    public static int pitchLastLeftAuto = 60;
    public static int rollLastLeftAuto = -190;
    //intake from the wall
    public static int pitchIntakeWall = 0;
    public static int rollIntakeWall = 10;

    //claw poses
    public static double clawOpen = .46;
    //public static double clawExtraOpen = 0;
    public static double clawClose = .7;

    //scoring
    public static int pitchWhenBasket = 0;
    public static int rollWhenBasket = 150;
    public static double pitchWhenHighChamber = 0;
    public static double rollWhenHighChamber = -20;
    public static double pitchFrontHighChamber = 40;
    public static double rollFrontHighChamber = 140;

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
    public static double armFrontHighChamberY = 18.8;
    //arm when high chamber
    public static double armHighChamberX = -3;
    public static double armHighChamberY = 18.5;

    //arm when intaking from sub
    public static double armReadySubIntakeX = 26;
    public static double armReadySubIntakeY = 10.5;
    public static double armSubIntakeY = 6;
    //arm when close (distance) intake
    public static double armCloseIntakeX = 15;
    public static double armCloseIntakeY = 8;
    //arm when intaking form the wall
    public static double armIntakeWallX = 7.6;
    public static double armIntakeWallY = 8.6;

    //manual arm boolean
    public static boolean manualArm = false;
    //arm auto park
    public static double armParkLeftAutoX = 0;
    public static double armParkLeftAutoY = 0;

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
    public static double translationKP = 0.03;
    public static double translationKI = 0.0;
    public static double translationKD = 0.0005;
    public static double headingKP = 0.008;
    public static double headingKI = 0.0;
    public static double headingKD = 0.0001;
    public static double lateralMutliplier = 1.5;

    public static double testX = 0.0;
    public static double testY = 0.0;
    public static double testHeading = 0;

}
