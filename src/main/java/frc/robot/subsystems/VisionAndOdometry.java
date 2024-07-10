// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class VisionAndOdometry extends SubsystemBase {
    private LimelightHelpers.PoseEstimate limelightPoseEstimation = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    private Debouncer goodUpdateFilter = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private Field2d field = new Field2d();
    
    private CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
        
    public SwerveDrivePoseEstimator robotPose = new SwerveDrivePoseEstimator(
        TunerConstants.kinematics, 
        drivetrain.getPigeon2().getRotation2d(),
        new SwerveModulePosition[] {
            TunerConstants.DriveTrain.getModule(0).getPosition(false),
            TunerConstants.DriveTrain.getModule(1).getPosition(false),
            TunerConstants.DriveTrain.getModule(2).getPosition(false),
            TunerConstants.DriveTrain.getModule(3).getPosition(false)
        },
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0))    
    );

    /** Creates a new Vision. */
    public VisionAndOdometry() {        
        SmartDashboard.putData("Field", field);
    }
    
    @Override
    public void periodic() {
        this.limelightPoseEstimation = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        updateOdometry();
        SmartDashboard.putNumber("Field_X", robotPose.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Field_Y", robotPose.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Field_Rot", robotPose.getEstimatedPosition().getRotation().getDegrees());
    }

    private boolean goodPoseEstimation() {
        boolean updateGood = true;

        if (this.getTranslation().getX() == 0.0)
            updateGood = false;

        if (this.getTranslation().getY() == 0.0)
            updateGood = false;

        if (this.getRotation2d().getDegrees() == 0.0)
            updateGood = false;

        return goodUpdateFilter.calculate(updateGood);
    }

    public Translation2d getTranslation() {
        return this.limelightPoseEstimation.pose.getTranslation();
    }

    public Rotation2d getRotation2d() {
        return this.limelightPoseEstimation.pose.getRotation();
    }

    public Pose2d getPose2d() {
        return this.limelightPoseEstimation.pose;
    }

    public double getPosTimeStamp() {
        return this.limelightPoseEstimation.timestampSeconds;
    }

    private void updateOdometry() {
        if (goodPoseEstimation()) {
            robotPose.addVisionMeasurement(getPose2d(), getPosTimeStamp());
        }

        robotPose.update(
            drivetrain.getPigeon2().getRotation2d(),
            new SwerveModulePosition[] {
                TunerConstants.DriveTrain.getModule(0).getPosition(false),
                TunerConstants.DriveTrain.getModule(1).getPosition(false),
                TunerConstants.DriveTrain.getModule(2).getPosition(false),
                TunerConstants.DriveTrain.getModule(3).getPosition(false)
            }
        );

        field.setRobotPose(
            robotPose.getEstimatedPosition().getX(), 
            robotPose.getEstimatedPosition().getY(), 
            robotPose.getEstimatedPosition().getRotation()
        );
    }

}
