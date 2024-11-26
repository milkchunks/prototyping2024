package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public abstract class CollectorConstants {
    public static final double PIVOT_ZERO_VOLTAGE = -2.0;
    public static final double ZERO_TIMEOUT = 5.0;
    public static final double VELOCITY_EPSILON = 5;
    public static final int REFRESH_RATE = 20;

    //10:72 driving a 12:36
    public static final double PIVOT_GEAR_REDUCTION = (10.0 / 72.0) * (16.0 / 36.0);
    public static final double SPIN_GEAR_REDUCTION = 18.0 / 36.0;

    //one second
    public static final double EJECT_DURATION = 1;

    //arbitrary for now
    //degrees from zero pos
    public static final double PIVOT_STOW_POS = 0;
    public static final double PIVOT_DEPLOY_POS = 110;
    public static final double PIVOT_EJECT_POS = 90;
    public static final double PIVOT_TOLERANCE = 0.5;


    public static final double PIVOT_STATOR_CURRENT_LIMIT = 1;
    public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 1;
    public static final double PIVOT_MAX_VELOCITY = 1;
    public static final double PIVOT_MAX_ACCELERATION = PIVOT_MAX_VELOCITY * 4;
    public static final double PIVOT_MAX_JERK = 0;
    //conversion to DEGREES
    public static final double PIVOT_SENSOR_TO_MECHANISM_RATIO = 1 / (PIVOT_GEAR_REDUCTION * 360);


    public static final double SPIN_STATOR_CURRENT_LIMIT = 30;
    public static final double SPIN_SUPPLY_CURRENT_LIMIT = 60;
    //both rpm i think
    public static final double SPIN_MAX_VELOCITY = 50;
    public static final double SPIN_MAX_ACCELERATION = SPIN_MAX_VELOCITY * 4;

    //conversion to degrees again i think (not super helpful but might need eventually)
    public static final double SPIN_SENSOR_TO_MECHANISM_RATIO = 1 / (SPIN_GEAR_REDUCTION * 360);

    //Deploying is positive
    public static final TalonFXConfiguration masterPivotMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withAudio(new AudioConfigs()
                    .withBeepOnConfig(true)
                    .withBeepOnBoot(true))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs() {{ContinuousWrap = false;}})
 //           .withCurrentLimits(new CurrentLimitsConfigs()
   //                 .withStatorCurrentLimit(PIVOT_STATOR_CURRENT_LIMIT)
//                    .withStatorCurrentLimitEnable(true)
   //                 .withSupplyCurrentLimitEnable(true))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(PIVOT_MAX_ACCELERATION)
                    .withMotionMagicCruiseVelocity(PIVOT_MAX_VELOCITY)
                    .withMotionMagicJerk(0.0))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(PIVOT_SENSOR_TO_MECHANISM_RATIO))
            .withSlot0(new Slot0Configs()
                    .withKP(0.1)
                    //.withKD(0)
                    .withKA(0.01)
                    .withKG(0.25)
                    .withKS(0.25)
                    .withKV(0.25));

    //Taking in is positive
    public static final TalonFXConfiguration spinMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withAudio(new AudioConfigs()
                    .withBeepOnConfig(true)
                    .withBeepOnBoot(true))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs() {{ContinuousWrap = false;}})
 //           .withCurrentLimits(new CurrentLimitsConfigs()
  //                  .withStatorCurrentLimit(SPIN_STATOR_CURRENT_LIMIT)
  //                  .withSupplyCurrentLimit(SPIN_SUPPLY_CURRENT_LIMIT)
  //                  .withStatorCurrentLimitEnable(true)
  //                  .withSupplyCurrentLimitEnable(true))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicJerk(PIVOT_MAX_JERK)
                    .withMotionMagicAcceleration(SPIN_MAX_ACCELERATION)
                    .withMotionMagicCruiseVelocity(SPIN_MAX_VELOCITY))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(SPIN_SENSOR_TO_MECHANISM_RATIO))
            .withSlot0(new Slot0Configs()
                    .withKP(0)
                    .withKD(0)
                    .withKA(0.01)
                    .withKG(0.25)
                    .withKS(0.25)
                    .withKV(0.25));
}