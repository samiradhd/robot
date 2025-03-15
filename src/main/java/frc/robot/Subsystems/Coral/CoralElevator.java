package frc.robot.Subsystems.Coral;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class CoralElevator implements ICoralElevatorIO, Subsystem{
    SparkMax elevator;
    SparkMaxConfig elevatorConfig;
    SparkClosedLoopController elevatorPID;
    RelativeEncoder relativeEncoder;
    Boolean homed;
    

    ElevatorStatesAutoLogged elevatorStates = new ElevatorStatesAutoLogged();

    public CoralElevator(int ElevatorMotorCANID){
        elevator = new SparkMax(ElevatorMotorCANID, MotorType.kBrushless);
        elevatorConfig = new SparkMaxConfig();
        elevatorPID = elevator.getClosedLoopController();
        relativeEncoder = elevator.getEncoder();

        elevatorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Coral.elevatorCurrentLimit);
        elevatorConfig.softLimit
            .forwardSoftLimit(Constants.Coral.l3Pos)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(Constants.Coral.intakePos)
            .reverseSoftLimitEnabled(true);
        elevatorConfig.encoder
            .positionConversionFactor(Constants.Coral.rotToM)
            .velocityConversionFactor(Constants.Coral.rotToM / 60);
            elevatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.Coral.positionPIDGains[0],Constants.Coral.positionPIDGains[1],Constants.Coral.positionPIDGains[2])
            .outputRange(-1,1);

        elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        homed = false;

    }
    @Override
    public void periodic() {
        
        updateStates(elevatorStates);
        Logger.processInputs("elevator", elevatorStates);
        // if (homed == false){
        // setElevatorPercentOutput(-4);
        //     if(elevator.getC)
        // }
    }
    

    @Override
    public ElevatorStatesAutoLogged updateStates(ElevatorStatesAutoLogged _elevatorStates) {
        
        _elevatorStates.OutputCurrent = elevator.getOutputCurrent();
        _elevatorStates.Temp = elevator.getMotorTemperature();

        _elevatorStates.elevatorPosition= getElevatorPosition();
        _elevatorStates.elevatorSpeed = getElevatorSpeed();
        
        return _elevatorStates;
    }

    public double getElevatorPosition(){
        return relativeEncoder.getPosition();
    }

    public double getElevatorSpeed(){
        return relativeEncoder.getVelocity();
    }

    public void setElevatorPercentOutput(double speed){
        elevator.set(speed);
    }

    public void resetEncoder(){
        relativeEncoder.setPosition(0);
    }

    public void setElevatorSpeed(double speed){
        elevatorPID.setReference(speed, ControlType.kVelocity);
    }

    public void setElevatorPosition(double height){
        elevatorPID.setReference(height, ControlType.kPosition);
    }
   
}
