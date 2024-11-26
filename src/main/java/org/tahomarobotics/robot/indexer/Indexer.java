package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;


//should probably have a listener
public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();
    public static Indexer getInstance() {
        return INSTANCE;
    }
    private final Logger logger = LoggerFactory.getLogger(this.getClass());
    private final DigitalInput collectorBreak = new DigitalInput(RobotMap.BEAM_BREAK_1);
    private final DigitalInput shooterBreak = new DigitalInput(RobotMap.BEAM_BREAK_2);
    private final TalonFX rollerMotor = new TalonFX(RobotMap.INDEXER_ROLLER);
    private IndexerState currentState = IndexerState.DISABLED;
    private final RobustConfigurator configurator = new RobustConfigurator(logger);
    private MotionMagicVelocityVoltage rpmController = new MotionMagicVelocityVoltage(0);

    public Indexer() {
        configurator.configureTalonFX(rollerMotor, IndexerConstants.rollerMotorConfig);
        BaseStatusSignal.setUpdateFrequencyForAll(IndexerConstants.REFRESH_RATE);
    }

    @Override
    public SubsystemIF initialize() {
        setName("Indexer");
        rollerMotor.setSafetyEnabled(false);
        return this;
    }

    @Override
    public void onTeleopInit() {
        setIndexerState(currentState);
    }

    @Override
    public void periodic() {
        if (!getCollectorBreakState() && getShooterBreakState()) {
            setIndexerState(IndexerState.INDEXING);
        } else if (!getCollectorBreakState() && !getShooterBreakState()) {
            System.out.println("COLLECTED!");
            setIndexerState(IndexerState.COLLECTED);
        }
        //if beam is broken, change state
    }

    public IndexerState getIndexerState() {
        return currentState;
    }

    public boolean getCollectorBreakState() {
        return collectorBreak.get();
    }

    public boolean getShooterBreakState() {
        return shooterBreak.get();
    }

    //TODO: do this
    public void setIndexerState(IndexerState newState) {
        switch (newState) {
            case DISABLED:
                stop();
                break;
            case INTAKING:
                //TODO: questionable
                setMotorSpeed(IndexerConstants.MAX_VELOCITY);
                break;
            case INDEXING:
                setMotorSpeed(IndexerConstants.INDEXING_VELOCITY);
                break;
            case COLLECTED:
                stop();
                break;
            case EJECTING:
                setMotorSpeed(-IndexerConstants.MAX_VELOCITY);
                break;
        }
        currentState = newState;
    }

    public enum IndexerState {
        DISABLED,
        INTAKING,
        INDEXING,
        COLLECTED,
        EJECTING;
    }


    //TODO: check
    public void setMotorSpeed(double rpm) {
        rollerMotor.setControl(rpmController.withVelocity(rpm));
    }

    //here for readability
    public void stop() {
        rollerMotor.stopMotor();
    }
}
