// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAtSpeaker;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.VisionAndOdometry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Lightning;


public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);

    public final Lightning leds = new Lightning(Constants.LedsProfile.id, Constants.LedsProfile.num_leds);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withDeadband(TunerConstants.maxAngularRate * 0.1)
        .withRotationalDeadband(TunerConstants.maxAngularRate * 0.1);

    private final Telemetry logger = new Telemetry(TunerConstants.maxSpeed);

    private final VisionAndOdometry visionAndOdometry = new VisionAndOdometry();

    private final Command aimAtSpeaker = new AimAtSpeaker(visionAndOdometry, driverController, fieldCentricDrive);


    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
                () -> fieldCentricDrive
                .withVelocityX(-driverController.getLeftY() * TunerConstants.maxSpeed)
                .withVelocityY(-driverController.getLeftX() * TunerConstants.maxSpeed)
                .withRotationalRate(-driverController.getRightX() * TunerConstants.maxAngularRate)
            )
        );
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        driverController.rightBumper().whileTrue(aimAtSpeaker);
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

}
