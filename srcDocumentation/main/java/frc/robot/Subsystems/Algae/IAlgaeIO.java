package frc.robot.Subsystems.Algae;

import org.littletonrobotics.junction.AutoLog;

public interface IAlgaeIO {
        /// more about IO and AutoLog on ICageIO.java

    @AutoLog
    public class AlgaeStates {

        double Temp;
        double OutputCurrent;
        double algaeAnglerPosition;
        double anglerSpeed;
        double intakePosition;
        double intakeSpeed;
    }

    public AlgaeStatesAutoLogged updateStates(AlgaeStatesAutoLogged _AlgaeStates);
}