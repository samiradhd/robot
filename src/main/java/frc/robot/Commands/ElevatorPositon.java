package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Coral.CoralElevator;
import frc.robot.Subsystems.Coral.CoralEndEffector;

public class ElevatorPositon extends Command{

    CoralElevator elevator;
    CoralEndEffector effector;
    Double setpoint;


    public ElevatorPositon(CoralElevator _elevator, Double _setpoint){
        this.elevator = _elevator;
        this.setpoint = _setpoint;
        addRequirements(this.elevator);

    }

    @Override
    public void execute() {
        this.elevator.setElevatorPosition(setpoint);       
    }

    @Override
    public boolean isFinished(){
        return Math.abs(elevator.getElevatorPosition() - setpoint) < 0.5;

    }


    @Override
    public void end(boolean interupted){
        System.out.println("end");
    }
    
}
