// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        FollowPathCommand.warmupCommand().schedule();
        this.robotContainer = new RobotContainer();
        this.robotContainer.leds.setDisabledLightShow();
        this.robotContainer.vision.setPipelineTo3d();
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().schedule(this.robotContainer.updatePosFromVision);
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        this.robotContainer.vision.setPipelineTo3d();
        this.robotContainer.leds.setDisabledLightShow();
    }

    @Override
    public void disabledPeriodic() {
        this.robotContainer.vision.setPipelineTo3d();
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        this.autonomousCommand = this.robotContainer.getAutonomousCommand();

        if (this.autonomousCommand != null) {
            this.autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
      if (this.autonomousCommand != null) {
            this.autonomousCommand.cancel();
        }
        this.robotContainer.leds.setTeleOpLightShow();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
