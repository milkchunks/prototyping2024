package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.SubsystemIF;

import static com.ctre.phoenix6.signals.ControlModeValue.Follower;

public class Collector extends SubsystemIF {
    //TODO :(
    private final Logger logger = LoggerFactory.getLogger(getInstance().getClass());
    private final TalonFX spinMotor = new TalonFX(RobotMap.COLLECTOR_SPIN_MOTOR);
    private final TalonFX pivotMotorL = new TalonFX(RobotMap.COLLECTOR_LEFT_PIVOT_MOTOR);
    private final TalonFX pivotMotorR = new TalonFX(RobotMap.COLLECTOR_RIGHT_PIVOT_MOTOR);
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(CollectorConstants.PIVOT_STOW_POS);

    private DeploymentState deployState = DeploymentState.STOWED;
    private CollectionState collectState = CollectionState.DISABLED;

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

    public void setPivotVoltage(double voltage) {
        pivotMotorL.setVoltage(voltage);
        pivotMotorR.setVoltage(voltage);
    }

    //TODO seems wrong because docs say pass in -1.0 to 1.0
    public void setSpinVelocity(double velocity) {
        if (velocity <= CollectorConstants.SPIN_MAX_VELOCITY) {
            spinMotor.set(velocity / CollectorConstants.SPIN_MAX_VELOCITY);
        } else {
            logger.warn("Value higher than spin max velocity has been passed into Collector.setSpinVelocity()");
        }
    }


    public void eject() {
        setDeploymentState(DeploymentState.EJECT);
        setCollectState(CollectionState.EJECTING);
    }

    public void setDeploymentState(DeploymentState newState) {
        switch (newState) {
            case STOWED:
                logger.info("Stowing collector...");
                zero();
                disable();
                deployState = DeploymentState.STOWED;
                break;
            case DEPLOYED:
                logger.info("Deploying collector...");
                pivotToPos(CollectorConstants.PIVOT_DEPLOY_POS);
                deployState = DeploymentState.DEPLOYED;
                break;
            case EJECT:
                logger.info("Ejecting...");
                pivotToPos(CollectorConstants.PIVOT_EJECT_POS);
                deployState = DeploymentState.EJECT;
                break;
        }
        logger.info("Success!!");
    }

    public DeploymentState getDeployState() {
        return deployState;
    }

    //TODO not all done
    public void setCollectState(CollectionState newState) {
        switch (newState) {
            case DISABLED:
                logger.info("Disabling...");
                disable();
                collectState = CollectionState.DISABLED;
                break;
            case COLLECTING:
                logger.info("Switching to collecting state...");
                setSpinVelocity(CollectorConstants.SPIN_MAX_VELOCITY);
                collectState = CollectionState.COLLECTING;
                break;
            case EJECTING:
                logger.info("Switching to ejecting state...");
                setSpinVelocity(-CollectorConstants.SPIN_MAX_VELOCITY);
                collectState = CollectionState.EJECTING;
                break;
        }
        logger.info("Success!!");
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

    //probably unnecessary but nice for readability
    private void disable() {
        pivotMotorL.disable();
        pivotMotorR.disable();
        spinMotor.disable();
    }

    public void zero() {
        pivotToPos(CollectorConstants.PIVOT_STOW_POS);
    }


    @Override
    public SubsystemIF initialize() {
        setDeploymentState(DeploymentState.STOWED);
        return this;
    }

    @Override
    public void periodic() {

    }
}
