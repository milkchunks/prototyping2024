package org.tahomarobotics.robot.indexer.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.commands.CollectorEjectCommand;
import org.tahomarobotics.robot.indexer.Indexer;

public class EjectCommand extends Command {
    private final Logger logger = LoggerFactory.getLogger(this.getClass());
    private final Indexer indexer = Indexer.getInstance();
    private final Timer timer = new Timer();
    private Indexer.IndexerState previousState;
    private final Collector collector = Collector.getInstance();

    public EjectCommand() {
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        timer.restart();
        logger.info("Indexer ejecting...");
        previousState = indexer.getIndexerState();
        System.out.println("indexer previous state" + previousState);
        if (previousState != Indexer.IndexerState.EJECTING) {
            indexer.setIndexerState(Indexer.IndexerState.EJECTING);
            CommandScheduler.getInstance().schedule(new CollectorEjectCommand());
        } else {
            logger.warn("Indexer was already ejecting when another eject command was scheduled. Aborting...");
            this.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return indexer.getCollectorBreakState() && indexer.getShooterBreakState();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setIndexerState(Indexer.IndexerState.DISABLED);
    }
}
