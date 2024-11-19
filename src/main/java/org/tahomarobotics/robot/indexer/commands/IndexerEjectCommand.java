package org.tahomarobotics.robot.indexer.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.indexer.Indexer;

public class IndexerEjectCommand extends Command {
    Logger logger = LoggerFactory.getLogger(this.getClass());
    Indexer indexer = Indexer.getInstance();
    Timer timer = new Timer();
    Indexer.IndexerState previousState;

    public IndexerEjectCommand() {
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        timer.restart();
        logger.info("Indexer ejecting...");
        previousState = indexer.getIndexerState();
        if (previousState != Indexer.IndexerState.EJECTING) {
            indexer.setIndexerState(Indexer.IndexerState.EJECTING);
        } else {
            logger.warn("Indexer was already ejecting when another eject command was scheduled. Aborting...");
            this.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return !indexer.getBeamBreak1State() && !indexer.getBeamBreak2State();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setIndexerState(Indexer.IndexerState.DISABLED);
    }
}
