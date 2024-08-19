// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TelopDrive extends Command {
    private Drivetrain drivetrain;
    private CommandXboxController driverController;
    private Vision vision;

    /** Creates a new TelopDrive. */
    public TelopDrive(Drivetrain drivetrain, CommandXboxController driverController, Vision vision) {
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        this.vision = vision;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.drivetrain);
        addRequirements(this.vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.vision.setPipelineTo3d();
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
        this.drivetrain.setVelocityAngular(-this.driverController.getHID().getRightX());
        if (this.vision.goodPoseEstimation())
            this.drivetrain.addVisionMeasurement(this.vision.getPose2d(), this.vision.getPosTimeStamp());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
