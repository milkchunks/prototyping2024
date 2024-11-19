package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.XboxController;
import org.tahomarobotics.robot.util.SubsystemIF;

public class OI extends SubsystemIF {
    private final XboxController driveController = new XboxController(RobotMap.CONTROLLER);
    private static final OI INSTANCE = new OI();

    public static OI getInstance() {
        return INSTANCE;
    }

    public OI() {

    }

    public int getPOV() {
        return driveController.getPOV();
    }

    public boolean getLeftBumper() {
        return driveController.getLeftBumperButton();
    }

    //maybe a default command?
    public boolean getLeftTrigger() {
        return triggerDeadband(driveController.getLeftTriggerAxis()) != 0;
    }

    private double triggerDeadband(double input) {
        if (input > 0.10) {
            return input;
        } else {
            return 0;
        }
    }
}
