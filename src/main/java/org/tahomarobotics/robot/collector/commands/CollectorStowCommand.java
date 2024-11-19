package org.tahomarobotics.robot.collector.commands;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorConstants;

public class CollectorStowCommand extends Command {
    Logger logger = LoggerFactory.getLogger(this.getClass());
    private final Collector collector = Collector.getInstance();

    public CollectorStowCommand() {
        addRequirements(collector);
    }

    //Set collector state to disabled and deploy state to stowed.
    @Override
    public void initialize() {
        logger.info("Stowing collector...");
        collector.setCollectState(Collector.CollectionState.DISABLED);
        collector.setDeploymentState(Collector.DeploymentState.STOWED);
    }

    //If collector is within tolerance for the zero position, end command.
    //If not, spin slowly forward/backward depending on if it undershot/overshot and end once the collector is within tolerance of the zero position.
    @Override
    public boolean isFinished() {
        double pivotPos = collector.getPivotPos().getValueAsDouble();
        if (Math.abs(pivotPos - CollectorConstants.PIVOT_STOW_POS) < CollectorConstants.PIVOT_TOLERANCE) {
            logger.info("Stowing success!");
            return true;
        } else {
            if (pivotPos <= 180) {
                logger.info("Stowing failure, undershot by " + (Math.abs(pivotPos - CollectorConstants.PIVOT_STOW_POS)) + " degrees.");
                collector.setPivotVoltage(-CollectorConstants.PIVOT_STOW_VOLTAGE);
            } else if (pivotPos > 180) {
                logger.info("Stowing failure, overshot by " + (Math.abs(pivotPos - CollectorConstants.PIVOT_STOW_POS)) + " degrees.");
                collector.setPivotVoltage(CollectorConstants.PIVOT_STOW_VOLTAGE);
            }
            return false;
        }
    }
}
