package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.armSubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenIntake;

import android.provider.Settings;
import android.util.Log;

import androidx.core.math.MathUtils;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.canvas.Rotation;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.other.Globals;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class VisionToSampleInterpolate extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private ArmSubsystem armSubsystem;

    private IntakeSubsystem intakeSubsystem;

    private BooleanSupplier slowMode;
    private DoubleSupplier strafe, forward, turn;


    public static double kPTurn = 0.005 * 180 / Math.PI *1.3;

    BasicPID turnpid = new BasicPID(new PIDCoefficients(kPTurn,0,0));

    private double prevSlidePosition = 0;

    private final double kPitchConversion = 2.33;

    private boolean offsetOnTarget = false;
    private boolean wristAngleOnTarget = false;

    private final double offsetTolerance = 7;

    private boolean hasFoundBlock = false;


    private InterpLUT lutXOffset = new InterpLUT(); //negative values report positive y poses
    private InterpLUT lutYOffset = new InterpLUT(); //

    private Pose2d samplePoseFieldOriented;

    double bruh = 0;

    boolean isAuto;

    double autoDesiredHeading;

    ElapsedTime timer = new ElapsedTime();

    public VisionToSampleInterpolate(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, Boolean isAuto, BooleanSupplier slowMode, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.slowMode=slowMode;
        this.strafe=strafe;
        this.forward=forward;
        this.turn=turn;

        this.isAuto = isAuto;

        addRequirements(visionSubsystem, armSubsystem, intakeSubsystem);

        if(!isAuto){
            addRequirements(driveSubsystem);
        }

        initializeLUTs();
    }

    public VisionToSampleInterpolate(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, Boolean isAuto){
        this(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, isAuto, ()->false, ()->0, ()->0, ()->0);
    }

    private void initializeLUTs(){
        lutXOffset.add(-9999,-7.5);
        lutXOffset.add(-138,-7.5);
        lutXOffset.add(-130.5,-7.0);
        lutXOffset.add(-121.5,-6.5);
        lutXOffset.add(-112,-6.0);
        lutXOffset.add(-103.5,-5.5);
        lutXOffset.add(-94,-5.0);
        lutXOffset.add(-85.5,-4.5);
        lutXOffset.add(-76.5,-4.0);
        lutXOffset.add(-67,-3.5);
        lutXOffset.add(-58,-3.0);
        lutXOffset.add(-49,-2.5);
        lutXOffset.add(-40,-2.0);
        lutXOffset.add(-30.5,-1.5);
        lutXOffset.add(-21,-1.0);
        lutXOffset.add(-11.5,-0.5);
        lutXOffset.add(0,0);
        lutXOffset.add(11.5,0.5);
        lutXOffset.add(21,1.0);
        lutXOffset.add(30.5,1.5);
        lutXOffset.add(40,2.0);
        lutXOffset.add(49,2.5);
        lutXOffset.add(58,3.0);
        lutXOffset.add(67,3.5);
        lutXOffset.add(76.5,4.0);
        lutXOffset.add(85.5,4.5);
        lutXOffset.add(94,5.0);
        lutXOffset.add(103.5,5.5);
        lutXOffset.add(112,6.0);
        lutXOffset.add(121.5,6.5);
        lutXOffset.add(130.5,7.0);
        lutXOffset.add(138,7.5);
        lutXOffset.add(9999,7.5);

        lutYOffset.add(-9999,6.0);
        lutYOffset.add(-100.5,6.0);
        lutYOffset.add(-92,5.5);
        lutYOffset.add(-83.5,5.0);
        lutYOffset.add(-76,4.5);
        lutYOffset.add(-68,4.0);
        lutYOffset.add(-60,3.5);
        lutYOffset.add(-52,3.0);
        lutYOffset.add(-42.5,2.5);
        lutYOffset.add(-36,2.0);
        lutYOffset.add(-25.5,1.5);
        lutYOffset.add(-17,1.0);
        lutYOffset.add(-8,0.5);
        lutYOffset.add(0,0);
        lutYOffset.add(10,-0.5);
        lutYOffset.add(20.5,1.0);
        lutYOffset.add(30.5,-1.5);
        lutYOffset.add(40.5,-2.0);
        lutYOffset.add(50.5,-2.5);
        lutYOffset.add(60.5,-3.0);
        lutYOffset.add(70.5,-3.5);
        lutYOffset.add(80,-4.0);
        lutYOffset.add(90,-4.5);
        lutYOffset.add(99,-5.0);
        lutYOffset.add(9999,-5.0);

        lutXOffset.createLUT();
        lutYOffset.createLUT();
    }

    @Override
    public void initialize(){
        intakeSubsystem.setDiffy(0,0);
        armSubsystem.setArmY(armReadySubIntakeY);
        hasFoundBlock=false;

        armSubsystem.setSlideP(0.15);
        visionSubsystem.turnOnStreaming(true);
        timer.reset();
    }

    @Override
    public void execute(){
        turnpid = new BasicPID(new PIDCoefficients(kPTurn,0,0));
        Optional<RotatedRect> allianceBoxFit = Optional.empty();
        if(!hasFoundBlock) {
            allianceBoxFit = visionSubsystem.getAllianceBoxFit();
        }

        if(allianceBoxFit.isPresent()&&!hasFoundBlock&&timer.milliseconds()>50){
            hasFoundBlock=true;

            List<Double> allianceOffsets = visionSubsystem.getOffsetFromBoxFit(allianceBoxFit.get());
            double xOffsetInches = lutXOffset.get(allianceOffsets.get(0));
            double yOffsetInches = lutYOffset.get(allianceOffsets.get(1));

            double allianceSkew = -visionSubsystem.getAngleFromRotatedRect(allianceBoxFit.get());

            Rotation2d allianceSkewRotation2d = new Rotation2d(Math.toRadians(allianceSkew));

            Transform2d cameraToSampleTransform = new Transform2d(new Translation2d(xOffsetInches,yOffsetInches), allianceSkewRotation2d);
            Transform2d robotToCameraTransform = new Transform2d(new Translation2d(0,armSubsystem.getCurrentX()-1.5), new Rotation2d());



            samplePoseFieldOriented = driveSubsystem.getPos().plus(robotToCameraTransform).plus(cameraToSampleTransform);

            intakeSubsystem.setDiffy(allianceSkew, rollWhenIntake);

            Translation2d botToSample = samplePoseFieldOriented.relativeTo(driveSubsystem.getPos()).getTranslation();

            autoDesiredHeading = Math.atan2(-botToSample.getX(), botToSample.getY());
        }

        if (hasFoundBlock){
            Translation2d botToSample = samplePoseFieldOriented.relativeTo(driveSubsystem.getPos()).getTranslation();


            //CCW is positive
            double headingErrorRadians  = Math.atan2(-botToSample.getX(), botToSample.getY());
            double slideExtension = botToSample.getNorm();
            double headingCalculation = turnpid.calculate(0, headingErrorRadians);
            double turnVelocity = Math.sqrt(Math.abs(headingCalculation)) * Math.signum(headingCalculation);
//            Log.i("stupidOmega", String.valueOf(turnVelocity));
            if(!isAuto){
                driveSubsystem.teleDrive(slowMode, true, 10, strafe.getAsDouble(), forward.getAsDouble(), turnVelocity);
            }
            else{
                driveSubsystem.readPinpoint();
                driveSubsystem.pidToRotation2d(new Rotation2d(autoDesiredHeading));
            }
            slideExtension = MathUtils.clamp(slideExtension, 7.75, 41);
            armSubsystem.setArmX(slideExtension);

            double wristAngle = samplePoseFieldOriented.relativeTo(driveSubsystem.getPos()).getRotation().getDegrees();


            //Technically could remove the ifs put i think its makes it more understandable
            if(wristAngle>=105){
                while(wristAngle>=105){
                    wristAngle-=180;
                }
            }
            else if (wristAngle<=-105){
                while(wristAngle<=-105){
                    wristAngle+=180;
                }
            }

            intakeSubsystem.setDiffy(-wristAngle*kPitchConversion);
        }
        else{
            if(!isAuto) {
                driveSubsystem.teleDrive(slowMode, true, 10, strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
            }
        }
    }

    @Override
    public void end(boolean e){
        armSubsystem.setSlideP(0.3);
        if(isAuto) {
            visionSubsystem.turnOnStreaming(false);
        }
    }

    @Override
    public boolean isFinished(){
//        return false;
        boolean driveOnTarget=false;
        if(isAuto){
            if(hasFoundBlock && Math.abs(driveSubsystem.getPos().getRotation().getRadians()-autoDesiredHeading)<Math.toRadians(5)){
                driveOnTarget=true;
                Log.i("errorAutoHeading", String.valueOf(Math.abs(Math.toDegrees(driveSubsystem.getPos().getRotation().getRadians()-autoDesiredHeading))));
                Log.i("errorAutoSlides", String.valueOf(Math.abs(armSubsystem.getTargetX()-armSubsystem.getSlideX())));
            }
        }

        boolean slidesOnTarget = hasFoundBlock && Math.abs(armSubsystem.getTargetX()-armSubsystem.getSlideX())<0.5 && Math.abs(armSubsystem.getSlideVelocity())<0.5;

        return driveOnTarget && slidesOnTarget;
    }
}
