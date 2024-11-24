package org.tahomarobotics.robot.indexer.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.indexer.Indexer;

public class IndexerIntakeCommand extends Command {
    Logger logger = LoggerFactory.getLogger(this.getClass());
    Indexer indexer = Indexer.getInstance();
    public IndexerIntakeCommand() {
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        logger.info("Indexer intaking...");
        indexer.setIndexerState(Indexer.IndexerState.INTAKING);
    }

    @Override
    public boolean isFinished() {
        return indexer.getBeamBreak2State();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setIndexerState(Indexer.IndexerState.DISABLED);
    }
}
