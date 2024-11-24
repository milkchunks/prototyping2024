package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.commands.CollectorEjectCommand;
import org.tahomarobotics.robot.collector.commands.CollectorZeroCommand;
import org.tahomarobotics.robot.indexer.commands.IndexerEjectCommand;
import org.tahomarobotics.robot.util.SubsystemIF;

public class OI extends SubsystemIF {
    private final CommandXboxController driveController = new CommandXboxController(RobotMap.CONTROLLER);
    private static final OI INSTANCE = new OI();
    public static OI getInstance() {
        return INSTANCE;
    }

    private final double TRIGGER_THRESHOLD = 0.10;
    private static Collector collector;

    public OI() {

    }

    @Override
    public SubsystemIF initialize() {
        collector = Collector.getInstance();
        setName("OI");
        bind();
        return this;
    }

    private void bind() {
        driveController.a().onTrue(new CollectorZeroCommand());
        driveController.leftTrigger(TRIGGER_THRESHOLD).whileTrue(collector.runOnce(collector::startCollecting));
        driveController.leftTrigger(TRIGGER_THRESHOLD).onFalse(collector.runOnce(collector::stop));
        driveController.povLeft().onTrue(new IndexerEjectCommand());
        driveController.leftBumper().onTrue(collector.runOnce(collector::toggleDeploy));
    }

    public double getLeftTrigger() {
        return driveController.getLeftTriggerAxis();
    }
}
