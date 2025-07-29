package frc.robot.Utilities;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

/// Our own wrapper of the XBOX Controller
/// The wrapper is to help make vibrating the controller easier
/// this should be pretty basic to understand as it is mostly just normal java not wpilib stuff.

public class Controller extends XboxController {
    public enum vibrationType {
        LEFT,
        RIGHT,
        BOTH;
    }

    public enum vibrationScalar {
        LOW,
        MEDIUM,
        HIGH;
    }

    
    public Controller(int port) {
        super(port);
    }

    public void setVibration(double value, GenericHID.RumbleType rumbleType) {
        setRumble(rumbleType, value);
    }

    /// just vibrates it for a set period
    private void vibrate(double value, double length, GenericHID.RumbleType rumbleType) {
        setVibration(value, rumbleType);
        ScheduledExecutorService schedulaService = Executors.newSingleThreadScheduledExecutor();

        schedulaService.schedule(() -> {
            setVibration(0, rumbleType);
            schedulaService.shutdown();
        }, 4, TimeUnit.SECONDS);
    }

    public void vibrate(double value, double length, vibrationType vibration) {
        vibrate(value, length, enumToRumbleType(vibration));
    }

    public void vibrate(vibrationScalar value, double length, vibrationType vibration) {
        vibrate(enumToRumbleValue(value), length, vibration);
    }

    public void vibrate(vibrationScalar value, vibrationType vibration) {
        vibrate(value, 1, vibration);
    }

    /// switches enum to the rumble type that is supported by XboxController
    public GenericHID.RumbleType enumToRumbleType(vibrationType vibration) {
        switch(vibration) {
            case LEFT:
               return GenericHID.RumbleType.kLeftRumble;
            case RIGHT:
                return GenericHID.RumbleType.kRightRumble;
            default:
                return GenericHID.RumbleType.kBothRumble;
        }
    }
    
    /// just switches enum to a double, giving the power of the vibration
    public double enumToRumbleValue(vibrationScalar vibration) {
        switch(vibration) {
            case LOW:
                return 0.2;
            case HIGH:
                return 1.0;
            default:
                return 0.5;
        }
    }
}
