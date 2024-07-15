// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionAndOdometry;


public class AimAtSpeaker extends Command {
    private VisionAndOdometry visionAndOdometry;
    private CommandXboxController driverController;
    private CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    private SwerveRequest.FieldCentric fieldCentricDrive;
    private double originalRotationDeadband;

    /** Creates a new AimAtSpeaker. */
    public AimAtSpeaker(VisionAndOdometry visionAndOdometry, CommandXboxController driverController, SwerveRequest.FieldCentric fieldCentricDrive) {
        this.visionAndOdometry = visionAndOdometry;
        this.driverController = driverController;
        this.fieldCentricDrive = fieldCentricDrive;
        
        // were going to override the rotation deadband, so bookmark it for later
        this.originalRotationDeadband = fieldCentricDrive.RotationalDeadband;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.drivetrain);
        addRequirements(this.visionAndOdometry);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // activate the speaker pipeline
        this.visionAndOdometry.setPipelineToSpeaker();

        // override the rotational deadband to allow the pid controller to do its thing
        this.fieldCentricDrive.withRotationalDeadband(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double pidOutput = this.visionAndOdometry.getSpeakerPIDOutput();

        // the drive train seems to keep the field relative request from robot container active
        // so for now, I'm going to just modify the local reference to that request, and let the drive train use it
        // this makes it work, but I don't like it...
        this.fieldCentricDrive = this.fieldCentricDrive
        .withVelocityX(-this.driverController.getLeftY() * TunerConstants.maxSpeed)
        .withVelocityY(-this.driverController.getLeftX() * TunerConstants.maxSpeed)
        .withRotationalRate(pidOutput - this.driverController.getRightX() * TunerConstants.maxAngularRate);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // this executes, i see the pipeline change back correctly
        this.visionAndOdometry.setPipelineTo3d();
        
        // restore the original deadband
        this.fieldCentricDrive.withDeadband(originalRotationDeadband);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
