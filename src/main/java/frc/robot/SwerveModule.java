// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
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
  GenericEntry drivePositionEntry;
  private static final double kModuleMaxAngularVelocity = DriveTrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  private final PWMEncoder m_turningEncoder;

  public static final double DT_WHEEL_DIAMETER = 0.10033;

  // Drive gear ratio (that number is the number of revolutions of the motor to get one revolution of the output)
  public static final double DT_DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  // Drive motor inverted
  public static final boolean DT_DRIVE_MOTOR_INVERTED = true;

  // Steer gear ratio (that number is the number of revolutions of the steer motor to get one revolution of the output)
  public static final double DT_STEER_GEAR_RATIO = 150.0 / 7.0;
  // Steer motor inverted
  public static final boolean DT_STEER_MOTOR_INVERTED = false;

  // Steer encoder gear ratio
  public static final double DT_STEER_ENCODER_GEAR_RATIO = 1;
  // Steer encoder inverted
  public static final boolean DT_STEER_ENCODER_INVERTED = false;
  
  private String name;
  private double offset;
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
      int turningEncoderChannelA, double offset) {
    this.name = name;
    this.offset = offset;
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);

    m_turningEncoder = new PWMEncoder(turningEncoderChannelA);
    speed = Shuffleboard.getTab("swerve").add(name + " speed", 0).getEntry();
    angle = Shuffleboard.getTab("swerve").add(name + " angle", 0).getEntry();
    drivePositionEntry  = Shuffleboard.getTab("swerve").add(name + " drivePostion", 0).getEntry();
    Shuffleboard.getTab("swerve").addDouble(name + "encoder", m_turningEncoder::getPosition);
    SwerveModule.DT_DRIVE_P.addChangeListener((value) -> {
      m_driveMotor.config_kP(0, value);
    });
    SwerveModule.DT_DRIVE_I.addChangeListener((value) -> {
      m_driveMotor.config_kI(0, value);
    });
    SwerveModule.DT_DRIVE_D.addChangeListener((value) -> {
      m_driveMotor.config_kD(0, value);
    });
    SwerveModule.DT_DRIVE_F.addChangeListener((value) -> {
      m_driveMotor.config_kF(0, value);
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
    SwerveModule.DT_STEER_F.addChangeListener((value) -> {
      m_turningMotor.config_kF(0, value);
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
    m_turningMotor.setSelectedSensorPosition(-angleToEncoderTicks(getAngle().getDegrees()));
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
    double revolutionsMotorToRevolutionsWheel = 1.0 / DT_DRIVE_GEAR_RATIO // Reduction from motor to output
    * (1.0 / (SwerveModule.DT_WHEEL_DIAMETER * Math.PI));

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
  public static final TunableDouble DT_STEER_F =
      new TunableDouble("DT_STEER_F", 0, "swerve").setSpot(3, 1);

  private double getDrivePosition() {
    double ticks = m_driveMotor.getSelectedSensorPosition();
    double ticksToRevolutions = 1.0 / 2048.0;
    double revolutionsMotorToRevolutionsWheel = 1.0 / DT_DRIVE_GEAR_RATIO // Reduction from motor to output
    * (1.0 / (SwerveModule.DT_WHEEL_DIAMETER * Math.PI));

    return ticks * ticksToRevolutions * revolutionsMotorToRevolutionsWheel;
  }

  private double mpsToEncoderTicks(double mps) {
    double sToMs = mps * 100.0 / 1000.0;
    double wheelRevolutions = sToMs / (DT_WHEEL_DIAMETER * Math.PI);
    double motorRev = wheelRevolutions * DT_DRIVE_GEAR_RATIO;
    double ticks = motorRev * 2048.0;
    return ticks; 
  }

  private double angleToEncoderTicks(double angle) {
    double angleToWheelRev = angle / 360.0;
    double motorRev = angleToWheelRev * DT_STEER_GEAR_RATIO;
    double ticks = motorRev * 2048.0;
    return ticks;
  }
  private double steerEncoderTicksToAngle(double ticks) {
    double motorRotation = ticks / 2048;
    double moduleRotation = motorRotation / DT_STEER_GEAR_RATIO;
    double angle = moduleRotation * 360;
    return angle;
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
    var rAngle = Rotation2d.fromDegrees(m_turningEncoder.getPosition() - offset);
    return Rotation2d.fromDegrees(MathUtil.inputModulus(rAngle.getDegrees(), 0, 360));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
drivePositionEntry.setDouble(getDrivePosition());

    Rotation2d rotation2d = Rotation2d.fromDegrees(steerEncoderTicksToAngle(-m_turningMotor.getSelectedSensorPosition()));
    // System.out.println("endoder:" + getAngle().getDegrees() + " | motor:" + rotation2d.getDegrees() + " | " + convertAngle( rotation2d.getDegrees(), 90));
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, rotation2d);
        // System.out.println(String.format("joystick:%.2f ", state.angle.getDegrees()) + String.format(" motor: %.2f ", rotation2d.getDegrees()) + String.format(" output: %.2f", convertAngle(rotation2d.getDegrees(), state.angle.getDegrees())));
        // speed.setDouble(mpsToEncoderTicks(state.speedMetersPerSecond));
    // angle.setDouble(angleToEncoderTicks(state.angle.getDegrees()));
    m_driveMotor.set(TalonFXControlMode.Velocity, mpsToEncoderTicks(state.speedMetersPerSecond) * -1);
    m_turningMotor.set(TalonFXControlMode.Position, angleToEncoderTicks(convertAngle(rotation2d.getDegrees(), state.angle.getDegrees()) * -1));

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
  public static double convertAngle(double start, double end){
    int angleRevo = (int)(start / 360);
    start %= 360;
    end %= 360;
    double change = start - end;
    if (change > 270) {
        end += 360;
    } else if (change < -270) {
        end -= 360;
    }
    end += 360 * angleRevo;
    return end;
  }
}
