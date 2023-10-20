// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.DriveTrain;
import frc.twilight.swerve.devices.tunables.TunableDouble;

public class SwerveModule {
  GenericEntry speed;
  GenericEntry angle;
  private static final double kModuleMaxAngularVelocity = DriveTrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  private final PWMEncoder m_turningEncoder;

  public static final double DT_WHEEL_DIAMETER = 0.10033;

  // Drive gear ratio
  public static final double DT_DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  // Drive motor inverted
  public static final boolean DT_DRIVE_MOTOR_INVERTED = true;

  // Steer gear ratio
  public static final double DT_STEER_GEAR_RATIO = 150.0 / 7.0;
  // Steer motor inverted
  public static final boolean DT_STEER_MOTOR_INVERTED = false;

  // Steer encoder gear ratio
  public static final double DT_STEER_ENCODER_GEAR_RATIO = 1;
  // Steer encoder inverted
  public static final boolean DT_STEER_ENCODER_INVERTED = false;

  // Steer CANcoder offset front left
  public static final double DT_FL_SE_OFFSET = 277;

  // Steer CANcoder offset front right
  public static final double DT_FR_SE_OFFSET = 124;

  // Steer CANcoder offset back left
  public static final double DT_BL_SE_OFFSET = 5;

  // Steer CANcoder offset back right
  public static final double DT_BR_SE_OFFSET = 248;

  


  private String name;
  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(
      String name, int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannelA) {
    this.name = name;
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);

    m_turningEncoder = new PWMEncoder(turningEncoderChannelA);
    speed = Shuffleboard.getTab("swerve").add(name + " speed", 0).getEntry();
    angle = Shuffleboard.getTab("swerve").add(name + " angle", 0).getEntry();
    SwerveModule.DT_DRIVE_P.addChangeListener((value) -> {
      m_driveMotor.config_kP(0, value);
    });
    SwerveModule.DT_DRIVE_I.addChangeListener((value) -> {
      m_driveMotor.config_kI(0, value);
    });
    SwerveModule.DT_DRIVE_D.addChangeListener((value) -> {
      m_driveMotor.config_kD(0, value);
    });
    SwerveModule.DT_STEER_P.addChangeListener((value) -> {
      m_turningMotor.config_kP(0, value);
    });
    SwerveModule.DT_STEER_I.addChangeListener((value) -> {
      m_turningMotor.config_kI(0, value);
    });
    SwerveModule.DT_STEER_D.addChangeListener((value) -> {
      m_turningMotor.config_kD(0, value);
    });
    
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), getAngle());
  }

  private double getDriveVelocity() {
    double ticks = m_driveMotor.getSelectedSensorVelocity();
    double msToS = 100.0 * (1.0 / 1000.0);
    double ticksToRevolutions = 1.0 / 2048.0;
    double revolutionsMotorToRevolutionsWheel = DT_DRIVE_GEAR_RATIO // Reduction from motor to output
    * (1 / (SwerveModule.DT_WHEEL_DIAMETER * Math.PI));

    return ticks * msToS * ticksToRevolutions * revolutionsMotorToRevolutionsWheel;
  }

  public static final TunableDouble DT_DRIVE_P =
      new TunableDouble("DT_DRIVE_P", 0.1, "swerve").setSpot(0, 0);
  public static final TunableDouble DT_DRIVE_I =
      new TunableDouble("DT_DRIVE_I", 0, "swerve").setSpot(1, 0);
  public static final TunableDouble DT_DRIVE_D =
      new TunableDouble("DT_DRIVE_D", 0.2, "swerve").setSpot(2, 0);
  public static final TunableDouble DT_DRIVE_F =
      new TunableDouble("DT_DRIVE_F", 0.052, "swerve").setSpot(3, 0);

  // PID values for the steer motor
  public static final TunableDouble DT_STEER_P =
      new TunableDouble("DT_STEER_P", 0.4, "swerve").setSpot(0, 1);
  public static final TunableDouble DT_STEER_I =
      new TunableDouble("DT_STEER_I", 0.0001, "swerve").setSpot(1, 1);
  public static final TunableDouble DT_STEER_D =
      new TunableDouble("DT_STEER_D", 1.274, "swerve").setSpot(2, 1);


  private double getDrivePosition() {
    double ticks = m_driveMotor.getSelectedSensorPosition();
    double ticksToRevolutions = 1.0 / 2048.0;
    double revolutionsMotorToRevolutionsWheel = DT_DRIVE_GEAR_RATIO // Reduction from motor to output
    * (1 / (SwerveModule.DT_WHEEL_DIAMETER * Math.PI));

    return ticks * ticksToRevolutions * revolutionsMotorToRevolutionsWheel;
  }

  private double mpsToEncoderTicks(double mps) {
    double sToMs = mps * 100 / 1000;
    double wheelRevolutions = mps / (DT_WHEEL_DIAMETER * Math.PI);
    double motorRev = wheelRevolutions / DT_WHEEL_DIAMETER;
    double ticks = motorRev * 2048;
    return ticks * sToMs; 
  }

  private double angleToEncoderTicks(double angle) {
    double angleToWheelRev = angle/ 360;
    double motorRev = angleToWheelRev / DT_STEER_GEAR_RATIO;
    double ticks = motorRev * 2048;
    return ticks;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDrivePosition(), getAngle());
  }

  private Rotation2d getAngle() {
    return new Rotation2d(m_turningEncoder.getPosition());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningMotor.getSelectedSensorPosition()));
    speed.setDouble(state.speedMetersPerSecond);
    angle.setDouble(state.angle.getDegrees());
    m_driveMotor.set(TalonFXControlMode.Velocity, mpsToEncoderTicks(state.speedMetersPerSecond));
    m_turningMotor.set(TalonFXControlMode.Position, angleToEncoderTicks(state.angle.getDegrees()));

    // Calculate the drive output from the drive PID controller.
    // final double driveOutput =
    //     m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    // final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // // Calculate the turning motor output from the turning PID controller.
    // final double turnOutput =
    //     m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

    // final double turnFeedforward =
    //     m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    // m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    // m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
