package org.firstinspires.ftc.teamcode.commands;

import androidx.core.math.MathUtils;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.other.Globals;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

@Config
public class VisionToSample extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private ArmSubsystem armSubsystem;

    private GamepadEx driver;
    private DoubleSupplier strafe, forward, turn;

    public static double kPSlides = 0.01;

    BasicPID slidePid = new BasicPID(new PIDCoefficients(kPSlides,0,0));


    public static double kPTurn = 0.005;

    BasicPID turnpid = new BasicPID(new PIDCoefficients(kPTurn,0,0));

    private double prevSlidePosition = 0;


    public VisionToSample(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, GamepadEx driver, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.armSubsystem = armSubsystem;

        this.driver=driver;
        this.strafe=strafe;
        this.forward=forward;
        this.turn=turn;

        addRequirements(driveSubsystem, visionSubsystem, armSubsystem);
    }

    @Override
    public void initialize(){
//        prevSlidePosition = armSubsystem.getSlideExtention();
        Globals.manualSlides=true;
    }

    @Override
    public void execute(){

        slidePid = new BasicPID(new PIDCoefficients(kPSlides,0,0));
        turnpid = new BasicPID(new PIDCoefficients(kPTurn,0,0));
        Optional<List<Double>> allianceOffsets = visionSubsystem.getAllianceOffsets();

        if(allianceOffsets.isPresent()){
            //ADJUST MIN AND MAX AS NECCESSARY
//            double slideCompensation = slidePid.calculate(0,allianceOffsets.get().get(1));
//            prevSlidePosition = MathUtils.clamp(prevSlidePosition + slideCompensation,15,30);
//            armSubsystem.setSlide(prevSlidePosition);

            armSubsystem.setSlidePower(MathUtils.clamp(-slidePid.calculate(allianceOffsets.get().get(1),0),-0.5,0.5));

            double omega = turnpid.calculate(allianceOffsets.get().get(0),0);
            driveSubsystem.teleDrive(driver, true, 10, strafe.getAsDouble(), forward.getAsDouble(), omega);
        }
        else {
            //Just asuuming the arctan arguements are constant, prob wont ever change
            driveSubsystem.teleDrive(driver, true, 10, strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
        }
    }

    public void end(boolean e){
        Globals.manualSlides=false;
        armSubsystem.setSlide(armSubsystem.getSlideExtention());
    }
}
