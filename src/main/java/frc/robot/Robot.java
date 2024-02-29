package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  PWMSparkMax left = new PWMSparkMax(0);
  PWMSparkMax right = new PWMSparkMax(1);

  Encoder leftEncoder = new Encoder(2, 3);
  Encoder rightEncoder = new Encoder(4, 5);

  EncoderSim simLeftEncoder = new EncoderSim(leftEncoder);
  EncoderSim simRightEncoder = new EncoderSim(rightEncoder);

  AnalogGyro gyro = new AnalogGyro(0);
  AnalogGyroSim simGyro = new AnalogGyroSim(gyro);

  Field2d field = new Field2d();

  XboxController xboxController = new XboxController(0);

  DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(1),
    7.29,
    7.5,
    60.0,
    Units.inchesToMeters(3),
    0.7112,
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  @Override
  public void robotInit() {
    
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    right.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    sim.setInputs(left.get() * RobotController.getBatteryVoltage(), right.get() * RobotController.getBatteryVoltage());
    sim.update(0.02);

    simLeftEncoder.setDistance(sim.getLeftPositionMeters());
    simRightEncoder.setDistance(sim.getRightPositionMeters());
    simLeftEncoder.setRate(sim.getLeftVelocityMetersPerSecond());
    simRightEncoder.setRate(sim.getRightVelocityMetersPerSecond());

    simGyro.setAngle(sim.getHeading().getDegrees());
    field.setRobotPose(sim.getPose());

    SmartDashboard.putData(field);
  }
}
