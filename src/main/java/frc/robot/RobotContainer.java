// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Logitech;

import static frc.robot.Constants.Control.*;
import static frc.robot.util.Logitech.Ports.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Drivetrain drivetrain = Drivetrain.getInstance();

  private Logitech driver = new Logitech(Driver.PORT);
  private JoystickButton dA = new JoystickButton(driver, A);
  private JoystickButton dB = new JoystickButton(driver, B);
  private JoystickButton dStart = new JoystickButton(driver, START);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureDefaultCommands();
    
    configureButtonBindings();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
      new RunCommand(
        () -> {
          // Drives the drivetrain
          drivetrain.drive(
            driver.getRawAxis(LEFT_STICK_X), 
            driver.getRawAxis(LEFT_STICK_Y), 
            driver.getRawAxis(RIGHT_STICK_X)
          );

          // Shifts up and down
          if (driver.getRawAxis(LEFT_TRIGGER) != 0.0) {
            drivetrain.shiftDown();
          } else if (driver.getRawAxis(RIGHT_TRIGGER) != 0.0) {
            drivetrain.shiftUp();
          }
        }, drivetrain
      ).withName("Drivetrain Default Command")
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Turn field centric on
    dA.whenPressed(
      new InstantCommand(
        () -> {
          drivetrain.setFieldCentricActive(true);
        }, drivetrain
      ).withName("Field Centric True")
    );
    // Turn field centric off
    dB.whenPressed(
      new InstantCommand(
        () -> {
          drivetrain.setFieldCentricActive(false);
        }, drivetrain
      ).withName("Field Centric False")
    );
    // Resets the gyro
    dStart.whenPressed(
      new InstantCommand(
        () -> {
          drivetrain.resetGyro();
        }, drivetrain
      ).withName("Reset Gyro")
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
