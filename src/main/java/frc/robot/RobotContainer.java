// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.VisionAndOdometry;
import frc.robot.subsystems.Lightning;


public class RobotContainer {
    private final CommandXboxController diverController = new CommandXboxController(0);

    public final Lightning leds = new Lightning(Constants.LedsProfile.id, Constants.LedsProfile.num_leds);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(TunerConstants.maxSpeed);

    public final VisionAndOdometry visionAndOdometry = new VisionAndOdometry();

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
                () -> new SwerveRequest.FieldCentric()
                .withDeadband(TunerConstants.maxAngularRate* 0.1)
                .withRotationalDeadband(TunerConstants.maxAngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withVelocityX(-diverController.getLeftY() * TunerConstants.maxSpeed)
                .withVelocityY(-diverController.getLeftX() * TunerConstants.maxSpeed)
                .withRotationalRate(-diverController.getRightX() * TunerConstants.maxAngularRate)
            )
        );
        diverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        diverController.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

}
