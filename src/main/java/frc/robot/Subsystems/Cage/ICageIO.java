package frc.robot.Subsystems.Cage;

import org.littletonrobotics.junction.AutoLog;

public interface ICageIO {

    /// EVERY subsystem has an interface.
    /// this is because we need to add states for each subsystem to diagnose them if messed up
    /// Autolog is a WPILIB Build feature that allows us to log these data
    /// recommended to look further on documentation but you just need to
    /// @AutoLog and have states for it to work
    /// you should be able to grasp onto this idea from just looking at the multiple subsystems

    @AutoLog
    public class CageStates {

        double Temp;
        double OutputCurrent;

        double cagePosition;
        double cageSpeed;
    }

    public CageStatesAutoLogged updateStates(CageStatesAutoLogged _CageStates);
}