package org.tahomarobotics.robot.collector.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorConstants;

public class CollectorEjectCommand extends Command {
    Logger logger = LoggerFactory.getLogger(this.getClass());
    private final Collector collector = Collector.getInstance();
    private final Timer timer = new Timer();
    private Collector.CollectionState previousCollectState = null;
    private Collector.DeploymentState previousDeployState = null;

    public CollectorEjectCommand() {
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        //Start timer
        timer.restart();
        logger.info("Ejecting...");

        //Store previous states so they can be returned back to when ejecting is done
        previousCollectState = collector.getCollectState();
        previousDeployState = collector.getDeployState();
        System.out.println("previous state: " + previousCollectState);
        System.out.println("previous state: " + previousDeployState);

        //If the state before ejecting was not already ejecting, finish ejecting and return to that state.
        //If the state before ejecting was somehow ejecting (should not be possible because creating a new eject command
        //is disabled while an eject command is already running), warn, disable and stow collector, then cancel command.
        if (previousCollectState != Collector.CollectionState.EJECTING && previousDeployState != Collector.DeploymentState.EJECT) {
            collector.eject();
        } else {
            logger.error("CollectorEjectCommand: State before ejecting was already ejecting. " +
                    "\nStates:" +
                    "\nPrevious collect state: " + previousCollectState.name() +
                    "\nPrevious deployment state: " + previousDeployState.name());
            collector.setCollectState(Collector.CollectionState.DISABLED);
            collector.setDeploymentState(Collector.DeploymentState.STOWED);
            this.cancel();
        }
    }

    //probably not a *great* idea to go by time in case the eject command gets stuck for some reason, then youre stuck for two seconds
    @Override
    public boolean isFinished() {
        //Stop once it's ejected for enough time
        return timer.hasElapsed(CollectorConstants.EJECT_DURATION);
    }

    //if interrupted i think it's fine because any other commands will change the state away from eject
    @Override
    public void end(boolean interrupted) {
        //Return to the state the collector was in before ejecting
        collector.setCollectState(previousCollectState);
        collector.setDeploymentState(previousDeployState);
    }
}
