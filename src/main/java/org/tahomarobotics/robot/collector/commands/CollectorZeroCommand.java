package org.tahomarobotics.robot.collector.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorConstants;

public class CollectorZeroCommand extends Command {
    private final Logger logger = LoggerFactory.getLogger(this.getClass());
    private final Timer timer = new Timer();
    private final Collector collector = Collector.getInstance();

    public CollectorZeroCommand() {
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        collector.setPivotVoltage(CollectorConstants.PIVOT_ZERO_VOLTAGE);
        SmartDashboard.putNumber("Zero Timer Value (Allison)", timer.get());
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(CollectorConstants.ZERO_TIMEOUT) || (timer.hasElapsed(0.25) && collector.getPivotVelocity() < CollectorConstants.VELOCITY_EPSILON);
    }

    //set position value to zero, then go to stow position
    @Override
    public void end(boolean interrupted) {
        collector.setDeploymentState(Collector.DeploymentState.STOWED);
        collector.zero();
    }
}
