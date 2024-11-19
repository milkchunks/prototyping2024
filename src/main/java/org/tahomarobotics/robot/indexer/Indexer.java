package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Indexer extends SubsystemIF {
    private final Logger logger = LoggerFactory.getLogger(this.getClass());
    private final DigitalInput beamBreak1 = new DigitalInput(RobotMap.BEAM_BREAK_1);
    private final DigitalInput beamBreak2 = new DigitalInput(RobotMap.BEAM_BREAK_2);
    private final TalonFX rollerMotor = new TalonFX(RobotMap.INDEXER_ROLLER);
    private IndexerState currentState = IndexerState.EMPTY;
    private final RobustConfigurator configurator = new RobustConfigurator(logger);

    private static final Indexer INSTANCE = new Indexer();
    public static Indexer getInstance() {
        return INSTANCE;
    }

    public Indexer() {
        configurator.configureTalonFX(rollerMotor, IndexerConstants.rollerMotorConfig);
    }

    @Override
    public SubsystemIF initialize() {
        setIndexerState(currentState);
        return this;
    }

    @Override
    public void periodic() {
        //if beam is broken, change state
    }

    public boolean getBeamBreak1State() {
        return beamBreak1.get();
    }

    public boolean getBeamBreak2State() {
        return beamBreak2.get();
    }

    public void setIndexerState(IndexerState newState) {
        switch (newState) {
            case EMPTY:
                break;
            case INTAKING:
                break;
            case INDEXING:
                break;
            case COLLECTED:
                break;
            case EJECTING:
                break;
        }
    }

    public enum IndexerState {
        EMPTY,
        INTAKING,
        INDEXING,
        COLLECTED,
        EJECTING;
    }
}
