package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public abstract class IndexerConstants {
    public static final TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
    public static final double REFRESH_RATE = 20;

    //rpm
    public static final double MAX_VELOCITY = 120;
    public static final double INDEXING_VELOCITY = MAX_VELOCITY / 10.0;
}
