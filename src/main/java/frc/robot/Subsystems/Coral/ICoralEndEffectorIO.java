package frc.robot.Subsystems.Coral;

import org.littletonrobotics.junction.AutoLog;

public interface ICoralEndEffectorIO {
    @AutoLog
    public class EffectorStates {
   
        double Temp;
        double OutputCurrent;

        double effectorPosition;
        double effectorSpeed;
    }

    public EffectorStatesAutoLogged updateStates(EffectorStatesAutoLogged _EffectorStates);
}