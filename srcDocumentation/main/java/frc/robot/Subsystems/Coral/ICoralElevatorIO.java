package frc.robot.Subsystems.Coral;

import org.littletonrobotics.junction.AutoLog;

public interface ICoralElevatorIO {
        /// more about IO and AutoLog on ICageIO.java

    @AutoLog
    public class ElevatorStates {

        double Temp;
        double OutputCurrent;

        double elevatorPosition;
        double elevatorSpeed;
    }

    public ElevatorStatesAutoLogged updateStates(ElevatorStatesAutoLogged _ElevatorStates);
}