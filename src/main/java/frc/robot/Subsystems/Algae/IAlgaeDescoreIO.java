package frc.robot.Subsystems.Algae;

import org.littletonrobotics.junction.AutoLog;

public interface IAlgaeDescoreIO {
    /// more about IO and AutoLog on ICageIO.java
    @AutoLog
    public class AlgaeDescoreStates {

        double Temp;
        double OutputCurrent;
        double algaeDescorePosition;
        double descoreSpeed;

    }

    public AlgaeDescoreStatesAutoLogged updateStates(AlgaeDescoreStatesAutoLogged _AlgaeDescoreStates);
}