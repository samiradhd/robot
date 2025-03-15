package frc.robot.Subsystems.Coral;

import org.littletonrobotics.junction.AutoLog;

public interface ICoralElevatorIO {
    @AutoLog
    public class ElevatorStates {
   
        double Temp;
        double OutputCurrent;

        double elevatorPosition;
        double elevatorSpeed;
    }

    public ElevatorStatesAutoLogged updateStates(ElevatorStatesAutoLogged _ElevatorStates);
}