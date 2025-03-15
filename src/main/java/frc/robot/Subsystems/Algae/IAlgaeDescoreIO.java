package frc.robot.Subsystems.Algae;

import org.littletonrobotics.junction.AutoLog;

public interface IAlgaeDescoreIO {
    @AutoLog
    public class AlgaeDescoreStates {
   
        double Temp;
        double OutputCurrent;
        double algaeDescorePosition;
        double descoreSpeed;
  
    }

    public AlgaeDescoreStatesAutoLogged updateStates(AlgaeDescoreStatesAutoLogged _AlgaeDescoreStates);
}