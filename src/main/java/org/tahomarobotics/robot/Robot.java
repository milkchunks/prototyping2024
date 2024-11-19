// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.commands.CollectorDeployCommand;
import org.tahomarobotics.robot.collector.commands.CollectorEjectCommand;
import org.tahomarobotics.robot.collector.commands.CollectorStowCommand;
import org.tahomarobotics.robot.util.SubsystemIF;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{ Logger logger = LoggerFactory.getLogger(this.getClass());

    private static final OI oi = OI.getInstance();
    private static final Collector collector = Collector.getInstance();
    
    SubsystemIF[] subsystems = {Collector.getInstance(), OI.getInstance()};
    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        for (int i = 0; i < subsystems.length; i++) {
            CommandScheduler.getInstance().registerSubsystem(subsystems[i].initialize());
        }
    }
    
    
    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {}
    
    
    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser, make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit()
    {
    }
    
    
    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
    }
    
    
    /** This method is called once when teleop is enabled. */
    @Override
    public void teleopInit() {}
    
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        if (oi.getPOV() == 270) {
            //...should be fine instead of runOnce(). probably.
            CommandScheduler.getInstance().schedule(new CollectorEjectCommand());
        } else if (oi.getLeftTrigger() && collector.getDeployState() == Collector.DeploymentState.DEPLOYED) {
            logger.info("Left trigger pressed, starting collector wheels.");
            Collector.getInstance().setCollectState(Collector.CollectionState.COLLECTING);
        } else if (oi.getLeftBumper()) {
            if (collector.getDeployState() == Collector.DeploymentState.DEPLOYED) {
                CommandScheduler.getInstance().schedule(new CollectorStowCommand());
            } else if (collector.getDeployState() == Collector.DeploymentState.STOWED) {
                CommandScheduler.getInstance().schedule(new CollectorDeployCommand());
            } else {
                logger.warn("State before toggling deploy/stow was neither deploy nor stow. " +
                        "\nState: " + collector.getDeployState().name());
                CommandScheduler.getInstance().schedule(new CollectorStowCommand());
            }
        }
    }    
    
    /** This method is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().schedule(new CollectorStowCommand());
    }
    
    
    /** This method is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}
    
    
    /** This method is called once when test mode is enabled. */
    @Override
    public void testInit() {}
    
    
    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
    
    
    /** This method is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}
    
    
    /** This method is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
