// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;


public class AimAtSpeaker extends Command {
    private Vision vision;
    private CommandXboxController driverController;
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.FieldCentric telopDrive;
    private PIDController angularController = Constants.SwerveProfile.angularController;

    /** Creates a new AimAtSpeaker. */
    public AimAtSpeaker(CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, Vision vision, SwerveRequest.FieldCentric telopDrive) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.driverController = driverController;
        this.telopDrive = telopDrive;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.drivetrain);
        addRequirements(this.vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // activate the speaker pipeline
        this.vision.setPipelineToSpeaker();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.angularController.setSetpoint(0);

        // we are mutating the telop drive request from robot container. applying new requests doesnt apply for whatever reason
        this.telopDrive
        .withVelocityX(-driverController.getLeftY() * Constants.SwerveProfile.maxSpeed)
        .withVelocityY(-driverController.getLeftX() * Constants.SwerveProfile.maxSpeed)
        .withRotationalRate(this.angularController.calculate(this.vision.getTx()) * Constants.SwerveProfile.maxAngularRate);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // restore the 3d pipeline
        this.vision.setPipelineTo3d();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
