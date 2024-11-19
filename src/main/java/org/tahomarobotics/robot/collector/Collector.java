package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.SubsystemIF;

import static com.ctre.phoenix6.signals.ControlModeValue.Follower;

public class Collector extends SubsystemIF {
    //TODO :(
    private static final Logger logger = LoggerFactory.getLogger(this.getClass());
    private static final TalonFX spinMotor = new TalonFX(RobotMap.COLLECTOR_SPIN_MOTOR);
    private static final TalonFX pivotMotorL = new TalonFX(RobotMap.COLLECTOR_LEFT_PIVOT_MOTOR);
    private static final TalonFX pivotMotorR = new TalonFX(RobotMap.COLLECTOR_RIGHT_PIVOT_MOTOR);
    MotionMagicVoltage positionControl = new MotionMagicVoltage(CollectorConstants.PIVOT_STOW_POS);

    private static DeploymentState deployState = DeploymentState.STOWED;
    private static CollectionState collectState = CollectionState.DISABLED;

    private static final Collector INSTANCE = new Collector();

    public static Collector getInstance() {
        return INSTANCE;
    }

    public Collector() {
        spinMotor.getConfigurator().apply(CollectorConstants.spinMotorConfig);
        pivotMotorL.getConfigurator().apply(CollectorConstants.masterPivotMotorConfig);
        pivotMotorR.getConfigurator().apply(CollectorConstants.followerPivotConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(CollectorConstants.REFRESH_RATE);
    }

    public StatusSignal<Angle> getPivotPos() {
        return pivotMotorL.getPosition();
    }

    public StatusSignal<Angle> getSpinPos() {
        return spinMotor.getPosition();
    }

    private void pivotToPos(double angle) {
        pivotMotorL.setControl(positionControl.withPosition(angle));
        pivotMotorR.setControl(positionControl.withPosition(angle));
    }

    //TODO seems wrong because docs say pass in -1.0 to 1.0
    public void setSpinVelocity(double velocity) {
        if (velocity <=CollectorConstants.SPIN_MAX_VELOCITY) {
            spinMotor.set(velocity / CollectorConstants.SPIN_MAX_VELOCITY);
        } else {

        }
    }


    //TODO do this
    public void eject() {

    }

    public void setDeploymentState(DeploymentState newState) {
        switch (newState) {
            case STOWED:
                pivotToPos(CollectorConstants.PIVOT_STOW_POS);
                deployState = DeploymentState.STOWED;
                break;
            case DEPLOYED:
                pivotToPos(CollectorConstants.PIVOT_DEPLOY_POS);
                deployState = DeploymentState.DEPLOYED;
                break;
            case EJECT:
                pivotToPos(CollectorConstants.PIVOT_EJECT_POS);
                deployState = DeploymentState.EJECT;
                break;
        }
    }

    public DeploymentState getDeployState() {
        return deployState;
    }

    //TODO not all done
    public void setCollectState(CollectionState newState) {
        switch (newState) {
            case DISABLED:
                collectState = CollectionState.DISABLED;
                break;
            case COLLECTING:
                collectState = CollectionState.COLLECTING;
                break;
            case EJECTING:
                collectState = CollectionState.EJECTING;
                break;
        }
    }

    public CollectionState getCollectState() {
        return collectState;
    }

    public enum CollectionState {
        DISABLED,
        COLLECTING,
        EJECTING;
    }

    public enum DeploymentState {
        STOWED,
        DEPLOYED,
        EJECT;
    }

    @Override
    public SubsystemIF initialize() {
        return this;
    }

    @Override
    public void periodic() {

    }
}
