package org.firstinspires.ftc.teamcode.other;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

@Config
public class PosGlobals {
    //starting pos
    public static final double startRightX = (3 + 7/16) + 14.74/2;//the 3&7/16 is the length a sample
    public static final double startLeftX = -startRightX;
//    public static final double startRightX = 14.74/2 + .5;//the 3&7/16 is the length a sample
//    public static final double startLeftX = -(2*(3 + 7/16) + 14.74/2);
    public static final double startLeftY = -70.9 + (8.18898);
    public static final double startRightY = startLeftY;
    public static Pose2d startingPosRight = new Pose2d(startRightX, startRightY, Rotation2d.fromDegrees(0));
    public static Pose2d startingPosLeft = new Pose2d(startLeftX, startLeftY, Rotation2d.fromDegrees(0));
    public static Pose2d startingPosLeft2 = new Pose2d(startLeftX - (3+7/16) - 23.75, startLeftY, Rotation2d.fromDegrees(0));


    //high chamber
    public static Pose2d highChamberLeft = new Pose2d(-5, -32.4, Rotation2d.fromDegrees(0));
    public static Pose2d firstHighChamberRight = new Pose2d(1, -33.0, Rotation2d.fromDegrees(0));
    public static Pose2d highChamberRight = new Pose2d(8, -32, Rotation2d.fromDegrees(180));
    public static Pose2d highChamberEvRight = new Pose2d(21, -40, Rotation2d.fromDegrees(39));


    //baskets
    public static Pose2d leftBasketPose = new Pose2d(-55, -55, Rotation2d.fromDegrees(-45));
    public static Pose2d leftBasketPose2 = new Pose2d(-56.5, -55, Rotation2d.fromDegrees(-45));

    //spikemarks
    public static Pose2d leftSideRightSpike = new Pose2d(-46, -39.75, Rotation2d.fromDegrees(0));
    public static Pose2d leftSideMidSpike = new Pose2d(-56, leftSideRightSpike.getY(), Rotation2d.fromDegrees(0));
    public static Pose2d leftSideLeftSpike = new Pose2d(-59.3, -36.85, Rotation2d.fromDegrees(35));

    public static Pose2d rightSideLeftSpike = new Pose2d(32, -37, Rotation2d.fromDegrees(-37));
    public static Pose2d rightSideMiddleSpike = new Pose2d(44, -36, Rotation2d.fromDegrees(-37));
    public static Pose2d rightSideRightSpike = new Pose2d(54, -36, Rotation2d.fromDegrees(-37));


    //observation zone pickup
    public static double obsZoneX = 0;
    public static double obsZoneY = 0;
    public static double obsZoneHeading = 0;



    //wallPickUp
    public static Pose2d wallPickUp = new Pose2d(38, -60 + 1, Rotation2d.fromDegrees(180));
    public static Pose2d wallEvPickUp = new Pose2d(38, -58, Rotation2d.fromDegrees(0));

    //parking
    public static Pose2d leftAutoPark = new Pose2d(-24, -7.09, Rotation2d.fromDegrees(-90));
}
