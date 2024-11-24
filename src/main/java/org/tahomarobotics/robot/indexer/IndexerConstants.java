package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public abstract class IndexerConstants {
    //intake is positive
    //rpm
    public static final double MAX_VELOCITY = 120;
    public static final double MAX_ACCELERATION = MAX_VELOCITY / 4;
    public static final double INDEXING_VELOCITY = MAX_VELOCITY / 10.0;

    //volts
    public static final double STATOR_LIMIT = 30;
    public static final double SUPPLY_LIMIT = 60;

    public static final TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withAudio(new AudioConfigs()
                    .withBeepOnBoot(true)
                    .withBeepOnBoot(true))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs() {{ContinuousWrap = false;}})
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(MAX_VELOCITY)
                    .withMotionMagicAcceleration(MAX_ACCELERATION))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(SUPPLY_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(STATOR_LIMIT)
                    .withStatorCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKA(0)
                    .withKG(0)
                    .withKS(0)
                    .withKV(0));
    public static final double REFRESH_RATE = 20;
}
