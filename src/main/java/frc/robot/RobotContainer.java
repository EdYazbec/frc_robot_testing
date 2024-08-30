// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Vision;
import frc.robot.commands.AimAtSpeaker;
import frc.robot.commands.UpdatePosFromVision;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Lightning;


public class RobotContainer {
    public final CommandXboxController driverController = new CommandXboxController(0);

    // Subsystems
    public final Lightning leds = new Lightning(Constants.LedsProfile.id, Constants.LedsProfile.num_leds);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    public final Vision vision = new Vision();

    // Commands
    private final SwerveRequest.FieldCentric telopDrive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.SwerveProfile.maxSpeed * 0.1)
      .withRotationalDeadband(Constants.SwerveProfile.maxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();
    private final Command aimAtSpeaker = new AimAtSpeaker(this.drivetrain, this.driverController, this.vision, this.telopDrive);
    public  final Command updatePosFromVision = new UpdatePosFromVision(this.drivetrain, this.vision).ignoringDisable(true);

    // path planner autos
    private final PathPlannerAuto centerNoteAuto = new PathPlannerAuto("CenterNote");
    private final SendableChooser<Command> pathChooser;

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> telopDrive
                .withVelocityX(-driverController.getLeftY() * Constants.SwerveProfile.maxSpeed)
                .withVelocityY(-driverController.getLeftX() * Constants.SwerveProfile.maxSpeed)
                .withRotationalRate(-driverController.getRightX() * Constants.SwerveProfile.maxAngularRate)
            )
        );
        driverController.a().whileTrue(drivetrain.applyRequest(() -> xBrake));
        driverController.rightBumper().whileTrue(this.aimAtSpeaker);
    }

    public RobotContainer() {
        // expose commands to the path planner gui
        NamedCommands.registerCommand("AimAtSpeaker", this.aimAtSpeaker);

        configureBindings();

        // build a sendable chooser for path planner paths
        pathChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoPathChooser", pathChooser);
    }

    public Command getAutonomousCommand() {
        return pathChooser.getSelected();
    }

}
