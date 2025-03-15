package frc.robot.Commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Coral.CoralElevator;
import frc.robot.Subsystems.Coral.CoralEndEffector;

public class CoralScore extends Command{

    CoralElevator elevator;
    CoralEndEffector effector;
    Double setpoint;


    public CoralScore(CoralElevator _elevator, CoralEndEffector _effector, Double _setpoint){
        this.elevator = _elevator;
        this.effector = _effector;
        this.setpoint = _setpoint;
        addRequirements(this.elevator, this.effector);

    }

    @Override
    public void initialize(){
        this.elevator.setElevatorPosition(setpoint);

    }
    
    @Override
    public void execute() {
        if(elevator.getElevatorPosition() + 0.5 >= setpoint){
            this.effector.setEffectorPercentOutput(-0.3);
        }
        SmartDashboard.putNumber("setpointElevator", setpoint);
    }

    @Override
    public boolean isFinished(){
        return this.effector.getTOFDistance() > Constants.Coral.outTakeTreshhold;

    }

    @Override
    public void end(boolean interupted){
        this.elevator.setElevatorPosition(Constants.Coral.intakePos);
        this.effector.setEffectorPercentOutput(0);

    }
    
}
