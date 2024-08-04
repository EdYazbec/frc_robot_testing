// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Vision;
import frc.robot.commands.AimAtSpeaker;
import frc.robot.commands.TelopDrive;
import frc.robot.commands.UpdatePosFromVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lightning;


public class RobotContainer {
    public final CommandXboxController driverController = new CommandXboxController(0);

    // Subsystems
    public final Lightning leds = new Lightning(Constants.LedsProfile.id, Constants.LedsProfile.num_leds);
    public final Drivetrain drivetrain = new Drivetrain();
    public final Vision vision = new Vision();

    // Commands
    private final Command telopDrive = new TelopDrive(this.drivetrain, this.driverController, this.vision);
    private final Command aimAtSpeaker = new AimAtSpeaker(this.drivetrain, this.driverController, this.vision);
    public  final Command updatePosFromVision = new UpdatePosFromVision(this.drivetrain, this.vision).ignoringDisable(true);

    private void configureBindings() {
        drivetrain.setDefaultCommand(telopDrive);
        driverController.a().onTrue(new InstantCommand(() -> this.drivetrain.setWheelBreak(true)));
        driverController.a().onFalse(new InstantCommand(() -> this.drivetrain.setWheelBreak(false)));
        driverController.y().onTrue(new InstantCommand(() -> this.drivetrain.resetHeading()));
        driverController.rightBumper().whileTrue(this.aimAtSpeaker);
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

}
