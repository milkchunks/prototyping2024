package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.OI;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.commands.CollectorDeployCommand;
import org.tahomarobotics.robot.collector.commands.CollectorStowCommand;
import org.tahomarobotics.robot.indexer.commands.IndexerEjectCommand;
import org.tahomarobotics.robot.indexer.commands.IndexerIntakeCommand;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static com.ctre.phoenix6.signals.ControlModeValue.Follower;

public class Collector extends SubsystemIF {
    //Singleton
    private static final Collector INSTANCE = new Collector();

    public static Collector getInstance() {
        return INSTANCE;
    }
    private OI oi;
    private final Logger logger = LoggerFactory.getLogger(this.getClass());
    private final TalonFX spinMotor = new TalonFX(RobotMap.COLLECTOR_SPIN_MOTOR);
    private final TalonFX pivotMotorL = new TalonFX(RobotMap.COLLECTOR_LEFT_PIVOT_MOTOR);
    private final TalonFX pivotMotorR = new TalonFX(RobotMap.COLLECTOR_RIGHT_PIVOT_MOTOR);
    //Motion magic for position with optional voltage parameter
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(CollectorConstants.PIVOT_STOW_POS);
    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);


    //Current deploy/collector state
    private DeploymentState deployState = DeploymentState.STOWED;
    private CollectionState collectState = CollectionState.DISABLED;
    private final RobustConfigurator configurator = new RobustConfigurator(logger);

    //Apply configs on load, and set update frequency to 20ms
    public Collector() {
        oi = oi.getInstance();
        setName("Collector");

        configurator.configureTalonFX(spinMotor, CollectorConstants.spinMotorConfig);
        configurator.configureTalonFX(pivotMotorL, CollectorConstants.masterPivotMotorConfig, pivotMotorR, true);

        BaseStatusSignal.setUpdateFrequencyForAll(CollectorConstants.REFRESH_RATE);
    }

    //Get pivot and spin position in degrees (i hope)
    public StatusSignal<Double> getPivotPos() {
        return pivotMotorL.getPosition();
    }

    public StatusSignal<Double> getSpinPos() {
        return spinMotor.getPosition();
    }

    //Use motion magic controller to pivot to a position in degrees
    private void pivotToPos(double angle) {
        pivotMotorL.setControl(positionControl.withPosition(angle));
    }

    //Used to spin for correcting zero command
    public void setPivotVoltage(double voltage) {
        pivotMotorL.setVoltage(voltage);
    }

    public void stop() {
        setCollectState(CollectionState.DISABLED);
    }

    public double getPivotVelocity() {
        return pivotMotorL.getVelocity().getValueAsDouble();
    }

    //TODO seems wrong because docs say pass in -1.0 to 1.0
    //Meters/second
    public void setSpinVelocity(double velocity) {
        if (velocity <= CollectorConstants.SPIN_MAX_VELOCITY) {
            spinMotor.set(velocity / CollectorConstants.SPIN_MAX_VELOCITY);
        } else {
            logger.warn("Value higher than spin max velocity has been passed into Collector.setSpinVelocity(), max velocity has been used instead.");
            spinMotor.set(CollectorConstants.SPIN_MAX_VELOCITY);
        }
    }


    //Deploy collector to eject position and spin rollers outwards
    public void eject() {
        setDeploymentState(DeploymentState.EJECT);
        setCollectState(CollectionState.EJECTING);
        //does not queue an indexer eject command because this method is only started by IndexerEjectCommand
    }

    //Change state
    //stow = zero pivot motors and disable collector spinny thing
    //deployed = pivot collector to be flipped out to collector notes
    //eject = hold collector a little bit above the ground to spit notes out
    public void setDeploymentState(DeploymentState newState) {
        switch (newState) {
            case STOWED:
                logger.info("Stowing collector...");
                stow();
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

    //Set collector spinny thing state
    //disable = stop collector spinny motor
    //collecting = spin inwards to take in notes
    //ejecting = spin outwards to spit out note
    public void setCollectState(CollectionState newState) {
        switch (newState) {
            case DISABLED:
                logger.info("Disabling...");
                spinMotor.disable();
                collectState = CollectionState.DISABLED;
                break;
            case COLLECTING:
                logger.info("Switching to collecting state...");
                setSpinVelocity(oi.getLeftTrigger());
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
    //Disable all collector motors
    private void disable() {
        pivotMotorL.disable();
        spinMotor.disable();
    }

    //Pivot to zero position
    public void zero() {
        pivotMotorL.setPosition(0);
    }


    //Stow and disable on initialize
    @Override
    public SubsystemIF initialize() {
        return this;
    }

    @Override
    public void onTeleopInit() {
        setDeploymentState(DeploymentState.STOWED);
        setCollectState(CollectionState.DISABLED);
    }

    //idk what to put here
    @Override
    public void periodic() {

    }

    //Only used to assign to controller in OI and probably isn't really
    //necessary, only exists for readability
    public void startCollecting() {
        logger.info("Starting collector motors...");
        setCollectState(CollectionState.COLLECTING);
        CommandScheduler.getInstance().schedule(new IndexerIntakeCommand());
    }




    //yet again for readabiliy in OI
    public void toggleDeploy() {
        if (deployState == DeploymentState.STOWED) {
            CommandScheduler.getInstance().schedule(new CollectorDeployCommand());
        } else if (deployState == DeploymentState.DEPLOYED) {
            CommandScheduler.getInstance().schedule(new CollectorStowCommand());
        }
    }

    private void stow() {
        pivotToPos(CollectorConstants.PIVOT_STOW_POS);
    }
}