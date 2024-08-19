// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;


public class Drivetrain extends SubsystemBase {
    public CommandSwerveDrivetrain swerveDrive = TunerConstants.DriveTrain;

    private SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withDeadband(TunerConstants.maxSpeed * 0.1)
        .withRotationalDeadband(TunerConstants.maxAngularRate * 0.1);

    private SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withDeadband(TunerConstants.maxSpeed * 0.1)
        .withRotationalDeadband(TunerConstants.maxAngularRate * 0.1);

    private SwerveRequest.SwerveDriveBrake breakRequest = new SwerveRequest.SwerveDriveBrake();

    private PIDController angularController = new PIDController(
        Constants.SwerveProfile.angularController.kP,
        Constants.SwerveProfile.angularController.kI,
        Constants.SwerveProfile.angularController.kD
    );

    private Telemetry logger = new Telemetry(TunerConstants.maxSpeed);

    private boolean brake = false;
    private boolean fieldCentric = true;
    private Field2d field2d = new Field2d();
    private boolean angularControllerOn = false;
    private double angularSetPoint = 0;
    private double velocityX = 0;
    private double velocityY = 0;
    private double velocityR = 0;

    /** Creates a new SwerveDrive. */
    public Drivetrain() {
        // configure the PathPlanner auto builder
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(
                        Constants.SwerveProfile.driveController.kP, 
                        Constants.SwerveProfile.driveController.kI, 
                        Constants.SwerveProfile.driveController.kD
                    ), // Translation PID constants
                    new PIDConstants(
                        Constants.SwerveProfile.angularController.kP,
                        Constants.SwerveProfile.angularController.kI,
                        Constants.SwerveProfile.angularController.kD
                    ), // Rotation PID constants
                    TunerConstants.maxSpeed, // Max module speed, in m/s
                    0.269, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> this.getIsRedAlliance(),
            this // Reference to this subsystem to set requirements
        );

        this.swerveDrive.registerTelemetry(logger::telemeterize);
        // this.resetHeading();
        SmartDashboard.putData("Field", this.field2d);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // if we do want to break
        if (this.brake) 
            this.swerveDrive.setControl(this.breakRequest);

        // if we don't want to break
        else {
            if (this.fieldCentric)
                driveFieldRelative();
            else
                driveRobotRelative();
        }

        field2d.setRobotPose(this.swerveDrive.getState().Pose);
    }

    private void driveFieldRelative() {
        // adjusting the field centric drive request        
        // if we are using the pid controller
        if (this.angularControllerOn) 
            this.fieldCentricDrive.withRotationalRate(this.angularController.calculate(angularSetPoint) * TunerConstants.maxAngularRate);

        // if are not using the PID controller
        else
            this.fieldCentricDrive.withRotationalRate(this.velocityR * TunerConstants.maxAngularRate);

        this.fieldCentricDrive.withVelocityX(this.velocityX * TunerConstants.maxSpeed);
        this.fieldCentricDrive.withVelocityY(this.velocityY * TunerConstants.maxSpeed);

        // apply the active swerve request
        this.swerveDrive.setControl(this.fieldCentricDrive);
    }

    /**
     * Configures the robot relative drive request for the drive train
     */
    private void driveRobotRelative() {
        // adjusting the field centric drive request
        // if we are using the pid controller
        if (this.angularControllerOn) 
            this.robotCentricDrive.withRotationalRate(this.angularController.calculate(angularSetPoint) * TunerConstants.maxAngularRate);

        // if are not using the PID controller
        else
            this.robotCentricDrive.withRotationalRate(this.velocityR * TunerConstants.maxAngularRate);

        this.robotCentricDrive.withVelocityX(this.velocityX * TunerConstants.maxSpeed);
        this.robotCentricDrive.withVelocityY(this.velocityY * TunerConstants.maxSpeed);

        // apply the active swerve request
        this.swerveDrive.setControl(this.robotCentricDrive);
    }

    /**
     * Sets the drive train to break stance
     * @param break turn break stance on if true, off if false
     */
    public void setWheelBreak(boolean brake) {
        this.brake = brake;
    }

    /**
     * Adds a vision measurement to the swerve drive odometry
     * @param position the pose2d of the robot
     * @param timestamp the time stamp of the vision measurement
     */
    public void addVisionMeasurement(Pose2d position, double timestamp) {
        // apply the vision measurement
        this.swerveDrive.addVisionMeasurement(position, timestamp);
    }

    /**
     * Sets the set point for the swerve drive's angular pid controller
     * @param setPoint set point for the controller
     */
    public void setAngularSetPoint(Double setPoint) {
        this.angularSetPoint = setPoint;
    }

    /**
     * Turns the swerve drive's angular controller on or off
     * @param controllerOn turn the controller on if true, off if false
     */
    public void setAngularControllerOnOff(boolean controllerOn) {
        this.angularControllerOn = controllerOn;
    }

    /**
     * Sets the x velocity of the swerve drive as a faction of the max speed 
     * @param velocity value between -1 and 1
     */
    public void setVelocityX(double velocity) {
        this.velocityX = velocity;
    }

    /**
     * Sets the y velocity of the swerve drive as a faction of the max speed 
     * @param velocity value between -1 and 1
     */
    public void setVelocityY(double velocity) {
        this.velocityY = velocity;
    }

    /**
     * Sets the angular velocity of the swerve drive as a faction of the max angular speed
     * @param velocity value between -1 and 1
     */
    public void setVelocityAngular(double velocity) {
        this.velocityR = velocity;
    }

    /**
     * Resents the "forward" direction of the robot
     * Note: vision localization will override this. If an april tag is in view, it will take priority.
     */
    public void resetHeading() {
        this.swerveDrive.seedFieldRelative();
    }

    /**
     * Toggles drive mode between field relative and robot relative.
     */
    public void toggleFieldRelative() {
        this.fieldCentric = !this.fieldCentric;
    }

    // public void setFieldCentric(boolean fieldCentric) {
    //     this.fieldCentric = fieldCentric;
    // }

    // these functions are for use by the path planner controller
    private Pose2d getPose() {
        // System.out.print("Get Pose: ");
        // System.out.println(this.swerveDrive.getState().Pose);
        return this.swerveDrive.getState().Pose;
    }

    private void resetPose(Pose2d pose) {
        this.swerveDrive.getState().Pose = pose;
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return this.swerveDrive.getState().speeds;
    }

    private boolean getIsRedAlliance() {
        if (DriverStation.isDSAttached()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                return true;
            else
                return false;
        }
        return false;
    }

    private void driveRobotRelative(ChassisSpeeds speeds) {
        System.out.println(speeds);
        this.fieldCentric = false;
        this.velocityX = speeds.vxMetersPerSecond;
        this.velocityY = speeds.vyMetersPerSecond;
        this.velocityR = speeds.omegaRadiansPerSecond;
    }

}
