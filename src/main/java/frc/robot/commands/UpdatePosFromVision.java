// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class UpdatePosFromVision extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private Vision vision;

    /** Creates a new UpdatePosFromVision. */
    public UpdatePosFromVision(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(this.vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.vision.setPipelineTo3d();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
