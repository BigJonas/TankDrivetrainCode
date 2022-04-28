// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMaxFactory;

import static frc.robot.Constants.Drivetrain.*;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain mInstance;

  private CANSparkMax l1;
  private CANSparkMax l2;
  private CANSparkMax r1;
  private CANSparkMax r2;
  private MotorControllerGroup l;
  private MotorControllerGroup r;
  private DifferentialDrive drive;

  private PIDController pidController;
  private SlewRateLimiter limiter;

  private DoubleSolenoid shifter;

  private AHRS gyro;

  private boolean fieldCentricActive;

  public synchronized static Drivetrain getInstance() {
    if (mInstance == null) {
      mInstance = new Drivetrain();
    }
    return mInstance;
  }

  /** Creates a new Drivetrain. */
  private Drivetrain() {
    l1 = SparkMaxFactory.createSparkMax(MOTOR_IDS[0], MOTOR_CONFIGS[0]);
    l2 = SparkMaxFactory.createSparkMax(MOTOR_IDS[1], MOTOR_CONFIGS[1]);
    r1 = SparkMaxFactory.createSparkMax(MOTOR_IDS[2], MOTOR_CONFIGS[2]);
    r2 = SparkMaxFactory.createSparkMax(MOTOR_IDS[3], MOTOR_CONFIGS[3]);
    l = new MotorControllerGroup(l1, l2);
    r = new MotorControllerGroup(r1, r2);
    drive = new DifferentialDrive(l, r);

    pidController = new PIDController(KP, KI, KD);
    limiter = new SlewRateLimiter(LIMITED_RATE_OF_CHANGE);

    shifter = new DoubleSolenoid(SHIFTER_MODULE_TYPE, SHIFTER_FORWARD_CHANNEL, SHIFTER_BACKWARD_CHANNEL);

    gyro = new AHRS(SerialPort.Port.kMXP);
    gyro.reset();
    gyro.setAngleAdjustment(0.0);

    fieldCentricActive = false;

    r.setInverted(true);

  }

  /**
   * Drives the robot in robot centric or field centric
   * @param xVelocity X Velocity of the stick
   * @param yVelocity Y Velocity of the stick
   * @param rot Rotational velocity (only used not in fieldCentric)
   */
  public synchronized void drive(double xVelocity, double yVelocity, double rot) {
    double fieldCentricAngleSetpoint = Math.atan2(xVelocity, yVelocity); // In radians    
    double fieldCentricSpeedSetpoint = Math.hypot(xVelocity, yVelocity); // From -1 to 1
    
    // TODO: I think this is 3:00 AM code this is a double nested terneary
    if (fieldCentricActive) {
      // Only drives when youre at the angle setpoint and rotates to the setpoint if youre not
      drive.arcadeDrive(
        // If it is within leftstick angling tolerance
        Math.abs(fieldCentricAngleSetpoint - getGyroAngle()) <= ROTATION_TOLERANCE ? 
          // If it is then drive with leftstick
          limiter.calculate(pidController.calculate(getWheelVelocity(), fieldCentricSpeedSetpoint)) :
          // If not then unable to drive  
          0.0,
        // If the right stick is being used
        Math.abs(rot) > 0.0 ? 
          // If true override lefstick  
          limiter.calculate(rot) : 
          // If not then if it is within leftstick angling tolerance
          Math.abs(fieldCentricAngleSetpoint - getGyroAngle()) <= ROTATION_TOLERANCE ? 
            // If its in tolerance than dont rotate
            0.0 : 
            // If else rotate using leftstick angling
            limiter.calculate(pidController.calculate(getGyroAngle(), fieldCentricAngleSetpoint)),
        true 
      );

    } else {
      // Just drives robot centric
      drive.arcadeDrive(
        limiter.calculate(xVelocity), 
        limiter.calculate(rot), 
        true
      );

    }

  }

  /**
   * Shifts the drivetrain up
   */
  public synchronized void shiftUp() {
    shifter.set(SHIFTER_UP_VALUE);
  }

  /**
   * Shifts the drivetrain down
   */
  public synchronized void shiftDown() {
    shifter.set(SHIFTER_DOWN_VALUE);
  }

  /**
   * Sets the state on whether or not the robot is in field centric
   * @param fieldCentricActive whether to activate field centric mode
   */
  public synchronized void setFieldCentricActive(boolean fieldCentricActive) {
    this.fieldCentricActive = fieldCentricActive;
  }

  /**
   * Gets the gyros angle
   * @return The angle of the gyro in radians
   */
  public double getGyroAngle() {
    return Math.toRadians(gyro.getAngle());
  }

  /**
   * Gets the velocity of the wheels
   * @return The velocity of the wheels in meters per second
   */
  public double getWheelVelocity() {
    return 
    (((r1.getEncoder().getVelocity() / 60) / TICKS_PER_REVOLUTION_LOW) * WHEEL_CIRCUMFERENCE_METERS + 
    ((-l1.getEncoder().getVelocity() / 60) / TICKS_PER_REVOLUTION_LOW) * WHEEL_CIRCUMFERENCE_METERS) / 2;
  }

  /**
   * Resets the gyro
   */
  public void resetGyro() {
    gyro.reset();
  }

  /**
   * Gets the gear of the drivetrain
   * @return The gear of the drivetrain, 1 if theyre in high gear and 0 if in low gear
   */
  public int getCurrentGear() {
    return shifter.get() == SHIFTER_UP_VALUE ? 1 : 0;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}