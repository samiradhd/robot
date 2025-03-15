package frc.robot.Subsystems.Algae;


import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Algae implements IAlgaeIO, Subsystem{

    SparkMax angler;
    SparkMaxConfig anglerconfig;
    SparkClosedLoopController anglerPID;
    RelativeEncoder anglerRelativeEncoder;

    SparkMax intake;
    SparkMaxConfig intakeconfig;
    SparkClosedLoopController intakePID;
    RelativeEncoder intakeRelativeEncoder;

    AlgaeStatesAutoLogged algaeStates = new AlgaeStatesAutoLogged();


    public Algae(int anglerMotorCANID, int intakeMotorCANID){
        angler = new SparkMax(anglerMotorCANID, MotorType.kBrushless);
        anglerconfig = new SparkMaxConfig();
        anglerPID = angler.getClosedLoopController();
        anglerRelativeEncoder = angler.getEncoder();
            

        anglerconfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Algae.anglerCurrentLimit);
        anglerconfig.softLimit
            .reverseSoftLimit(0)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(75)
            .forwardSoftLimitEnabled(true);
        anglerconfig.encoder
            .positionConversionFactor(Constants.Algae.rotToDeg)
            .velocityConversionFactor(Constants.Algae.rotToDeg/ 60);
        anglerconfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-0.5,0.5)
            .pid(0.015,0,0);

        angler.configure(anglerconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


       
        intake = new SparkMax(intakeMotorCANID, MotorType.kBrushless);
        intakeconfig = new SparkMaxConfig();
        intakePID = intake.getClosedLoopController();
        intakeRelativeEncoder = intake.getEncoder();

        intakeconfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Algae.anglerCurrentLimit);
        intakeconfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(0.002,0,0, 0.0001);            

        intake.configure(intakeconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        updateStates(algaeStates);
        Logger.processInputs("algae", algaeStates);
    }

    @Override
    public AlgaeStatesAutoLogged updateStates(AlgaeStatesAutoLogged _algaeStates) {
        
        _algaeStates.OutputCurrent = angler.getOutputCurrent();
        _algaeStates.Temp = angler.getMotorTemperature();

        _algaeStates.algaeAnglerPosition= getAnglerPosition();
        _algaeStates.anglerSpeed = getAnglerSpeed();

        _algaeStates.intakeSpeed = getIntakeSpeed();
        
        return _algaeStates;
    }

    public double getAnglerPosition(){
        return anglerRelativeEncoder.getPosition();
    }

    public double getAnglerSpeed(){
        return anglerRelativeEncoder.getVelocity();
    }

    public double getIntakeSpeed(){
        return intakeRelativeEncoder.getVelocity();
    }

    public double getIntakeCurrent(){
        return intake.getOutputCurrent();
    }

    public void setAnglerPercentOutput(double speed){
        angler.set(speed);
    }

    public void setIntakePercentOutput(double speed){
        intake.set(speed);
    }

    public void setIntakeSpeed(double intakeSpeed){
        intakePID.setReference(intakeSpeed, ControlType.kVelocity);
    }

    public void setAnglerPosition(double angle){
        anglerPID.setReference(angle, ControlType.kPosition);
    }
   
}

    

