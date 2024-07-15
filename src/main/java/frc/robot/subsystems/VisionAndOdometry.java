// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


public class VisionAndOdometry extends SubsystemBase {
    // a PID controller for targeting at the speaker
    private PIDController speakerPID = new PIDController(
        Constants.VisionProfile.kp,
        Constants.VisionProfile.ki,
        Constants.VisionProfile.kd
    );

    // an object to handle position estimation using the drive train and vision updates
    public SwerveDrivePoseEstimator robotPose = new SwerveDrivePoseEstimator(
        TunerConstants.kinematics, 
        TunerConstants.DriveTrain.getPigeon2().getRotation2d(),
        new SwerveModulePosition[] {
            TunerConstants.DriveTrain.getModule(0).getPosition(false),
            TunerConstants.DriveTrain.getModule(1).getPosition(false),
            TunerConstants.DriveTrain.getModule(2).getPosition(false),
            TunerConstants.DriveTrain.getModule(3).getPosition(false)
        },
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0))
    );

    // objects to facilitate position estimates from limelight 
    private LimelightHelpers.PoseEstimate limelightPoseEstimation = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.VisionProfile.limelightName);
    private Debouncer goodUpdateFilter = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    private Field2d field = new Field2d();

    // The pipeline the limelight should use
    private int speakerPipeline;

    /** Creates a new VisionAndOdometry. */
    public VisionAndOdometry() {        
        SmartDashboard.putData("Field", field);
        
        checkAlliance();

        // set the starting pipeline to 3d pos to start
        setPipeline(Constants.VisionProfile.posFromSpeakerPipeline);
    }
    
    @Override
    public void periodic() {
        this.limelightPoseEstimation = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.VisionProfile.limelightName);
        updateOdometry();
        SmartDashboard.putNumber("Field_X", robotPose.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Field_Y", robotPose.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Field_Rot", robotPose.getEstimatedPosition().getRotation().getDegrees());
    }

    /**
     * Returns true if the there is a good vision update
     * @return bool
     */
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

    /**
     * Returns the 2d translation from the limelight position estimation object.
     * @return Translation2d
     */
    private Translation2d getTranslation() {
        return this.limelightPoseEstimation.pose.getTranslation();
    }

    /**
     * Returns the 2d rotation from the limelight position estimation object.
     * @return Rotation2d
     */
    private Rotation2d getRotation2d() {
        return this.limelightPoseEstimation.pose.getRotation();
    }

    /**
     * Returns the 2d position from the limelight position estimation object.
     * @return Pose2d
     */
    private Pose2d getPose2d() {
        return this.limelightPoseEstimation.pose;
    }

    /**
     * Returns the timestamp in seconds from the limelight position estimation object.
     * @return double
     */
    private double getPosTimeStamp() {
        return this.limelightPoseEstimation.timestampSeconds;
    }

    /**
     * Updates the robotPose object if the most recent position estimation from the limelight was good.
     */
    private void updateOdometry() {
        if (goodPoseEstimation()) {
            robotPose.addVisionMeasurement(getPose2d(), getPosTimeStamp());
        }

        robotPose.update(
            TunerConstants.DriveTrain.getPigeon2().getRotation2d(),
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

    /**
     * Checks the driver station for which alliance we are and updates the pipeline for speaker targeting accordingly.
     */
    private void checkAlliance() {
        // Check which alliance we are for to target the correct speaker
        if (DriverStation.isDSAttached()) {
            if (DriverStation.getAlliance().get() == Alliance.Red)
                this.speakerPipeline = Constants.VisionProfile.redSpeakerPipeline;
            else
                this.speakerPipeline = Constants.VisionProfile.blueSpeakerPipeline;
        }
        else
            this.speakerPipeline = Constants.VisionProfile.blueSpeakerPipeline;
    }

    /**
     * Returns the angular offset from the target in x direction from the limelight in degrees.
     * @return double
     */
    private double getTx() {
        return LimelightHelpers.getTX(Constants.VisionProfile.limelightName);
    }

    /**
     * Sets the limelight pipeline
     * @param pipeline limelight pipeline to make active
     */
    private void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(Constants.VisionProfile.limelightName, pipeline);
    }

    /**
     * Sets the limelight pipeline for 3d position tracking
     */
    public void setPipelineTo3d() {
        setPipeline(Constants.VisionProfile.posFromSpeakerPipeline);
    }

    /**
     * Sets the limelight pipeline for speaker specific tracking
     */
    public void setPipelineToSpeaker() {
        checkAlliance();
        setPipeline(this.speakerPipeline);
    }
    /**
     * Updates the speaker tracker PID controller and returns its output.
     * Sets the limelight pipelines to track the speaker, be sure to call setPipelineTo3d to restore 3d position pipeline.
     * @return double
     */
    public double getSpeakerPIDOutput() {
        return this.speakerPID.calculate(getTx());
    }
}
