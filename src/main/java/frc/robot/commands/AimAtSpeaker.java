// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;


public class AimAtSpeaker extends Command {
    private Vision vision;
    private CommandXboxController driverController;
    private Drivetrain drivetrain;

    /** Creates a new AimAtSpeaker. */
    public AimAtSpeaker(Drivetrain drivetrain, Vision vision, CommandXboxController driverController) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.driverController = driverController;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.drivetrain);
        addRequirements(this.vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // activate the speaker pipeline
        this.vision.setPipelineToSpeaker();
        this.drivetrain.setAngularControllerOnOff(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // xy controls need to be flipped if we are red / blue
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            this.drivetrain.setVelocityX(-this.driverController.getHID().getLeftY());
            this.drivetrain.setVelocityY(-this.driverController.getHID().getLeftX());
        }
        else {
            this.drivetrain.setVelocityX(this.driverController.getHID().getLeftY());
            this.drivetrain.setVelocityY(this.driverController.getHID().getLeftX());
        }
        this.drivetrain.setAngularSetPoint(this.vision.getTx());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // restore the 3d pipeline
        this.vision.setPipelineTo3d();
        this.drivetrain.setAngularControllerOnOff(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
