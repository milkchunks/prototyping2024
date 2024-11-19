package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public abstract class CollectorConstants {
    public static final int REFRESH_RATE = 20;


    //arbitrary for now
    //degrees from zero pos
    public static final double PIVOT_STOW_POS = 0;
    public static final double PIVOT_DEPLOY_POS = 178;
    public static final double PIVOT_EJECT_POS = 170;


    public static final double PIVOT_STATOR_CURRENT_LIMIT = 1;
    public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 1;
    public static final double PIVOT_MAX_ACCELERATION = 1;
    public static final double PIVOT_MAX_VELOCITY = 1;
    public static final double PIVOT_SENSOR_TO_MECHANISM_RATIO = 1;


    public static final double SPIN_STATOR_CURRENT_LIMIT = 1;
    public static final double SPIN_SUPPLY_CURRENT_LIMIT = 1;
    public static final double SPIN_MAX_ACCELERATION = 1;
    public static final double SPIN_MAX_VELOCITY = 1;
    public static final double SPIN_SENSOR_TO_MECHANISM_RATIO = 1;

    //Deploying is positive
    public static final TalonFXConfiguration masterPivotMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withAudio(new AudioConfigs()
                    .withBeepOnConfig(true)
                    .withBeepOnBoot(true))
            //.withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                    //.ContinuousWrap)
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(PIVOT_STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(PIVOT_SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(PIVOT_MAX_ACCELERATION)
                    .withMotionMagicCruiseVelocity(PIVOT_MAX_VELOCITY))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(PIVOT_SENSOR_TO_MECHANISM_RATIO))
            .withSlot0(new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKA(0)
                    .withKG(0)
                    .withKS(0)
                    .withKV(0));

    public static final TalonFXConfiguration followerPivotConfig = masterPivotMotorConfig
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive));


    //Taking in is positive
    public static final TalonFXConfiguration spinMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withAudio(new AudioConfigs()
                    .withBeepOnConfig(true)
                    .withBeepOnBoot(true))
            //.withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
            //.ContinuousWrap)
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(SPIN_STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(SPIN_SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(SPIN_MAX_ACCELERATION)
                    .withMotionMagicCruiseVelocity(SPIN_MAX_VELOCITY))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(SPIN_SENSOR_TO_MECHANISM_RATIO))
            .withSlot0(new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKA(0)
                    .withKG(0)
                    .withKS(0)
                    .withKV(0));
}