package org.tahomarobotics.robot.collector.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorConstants;

//Built like stow command but it deploys instead
public class CollectorDeployCommand extends Command {
    Logger logger = LoggerFactory.getLogger(this.getClass());
    Collector collector = Collector.getInstance();

    public CollectorDeployCommand() {
        addRequirements(collector);
    }

    //Set deploy state to deployed.
    @Override
    public void initialize() {
        logger.info("Deploying collector...");
        collector.setDeploymentState(Collector.DeploymentState.DEPLOYED);
    }

    //If collector is within tolerance for the zero position, end command.
    //If not, spin slowly forward/backward depending on if it undershot/overshot
    //and end once the collector is within tolerance of the zero position.
    @Override
    public boolean isFinished() {
        double pivotPos = collector.getPivotPos().getValueAsDouble();
        if (Math.abs(pivotPos - CollectorConstants.PIVOT_DEPLOY_POS) < CollectorConstants.PIVOT_TOLERANCE) {
            logger.info("Deploy success!");
            return true;
        } else {
            if (pivotPos <= CollectorConstants.PIVOT_DEPLOY_POS) {
                logger.info("Deploy failure, undershot by " + (Math.abs(pivotPos - CollectorConstants.PIVOT_DEPLOY_POS)) + " degrees.");
                collector.setPivotVoltage(-CollectorConstants.PIVOT_CORRECTION_VOLTAGE);
            } else if (pivotPos > CollectorConstants.PIVOT_DEPLOY_POS) {
                logger.info("Deploy failure, overshot by " + (Math.abs(pivotPos - CollectorConstants.PIVOT_DEPLOY_POS)) + " degrees.");
                collector.setPivotVoltage(CollectorConstants.PIVOT_CORRECTION_VOLTAGE);
            }
            return false;
        }
    }
}
