// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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

    private SwerveRequest.SwerveDriveBrake breakRequest = new SwerveRequest.SwerveDriveBrake();

    private PIDController angularController = new PIDController(
        Constants.SwerveProfile.angularController.kP,
        Constants.SwerveProfile.angularController.kI,
        Constants.SwerveProfile.angularController.kD
    );
    private Telemetry logger = new Telemetry(TunerConstants.maxSpeed);

    private boolean brake = false;
    private Field2d field2d = new Field2d();
    private boolean angularControllerOn = false;
    private double angularSetPoint = 0;
    private double velocityX = 0;
    private double velocityY = 0;
    private double velocityR = 0;

    /** Creates a new SwerveDrive. */
    public Drivetrain() {
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
    }

    public void setWheelBreak(boolean brake) {
        this.brake = brake;
    }

    public void addVisionMeasurement(Pose2d position, double timestamp) {
        // apply the vision measurement
        this.swerveDrive.addVisionMeasurement(position, timestamp);

        // update the field map on smartdashbord
        field2d.setRobotPose(this.swerveDrive.getState().Pose);
    }

    public void setAngularSetPoint(Double setPoint) {
        this.angularSetPoint = setPoint;
    }

    public void setAngularControllerOnOff(boolean controllerOn) {
        this.angularControllerOn = controllerOn;
    }

    public void setVelocityX(double velocity) {
        this.velocityX = velocity;
    }

    public void setVelocityY(double velocity) {
        this.velocityY = velocity;
    }

    public void setVelocityAngular(double velocity) {
        this.velocityR = velocity;
    }

    public void resetHeading() {
        this.swerveDrive.seedFieldRelative();
    }
}
