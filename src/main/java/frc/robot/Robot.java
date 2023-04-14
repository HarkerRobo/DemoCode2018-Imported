// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  XboxController controller = new XboxController(0);
  XboxController operator = new XboxController(1);
  TalonSRX leftMaster, rightMaster, intakeMaster, elevatorMaster, intakeFollower;
  VictorSPX leftFollower, rightFollower, elevatorFollowerA, elevatorFollowerB, elevatorFollowerC;
  DoubleSolenoid intakePiston, raisePiston;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    leftMaster = new TalonSRX(0);
    rightMaster = new TalonSRX(1);
    leftFollower = new VictorSPX(0);
    rightFollower = new VictorSPX(1);
    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightFollower.configFactoryDefault();
    
		// invert
		leftMaster.setInverted(true);
		leftFollower.setInverted(true);
		rightFollower.setInverted(false);
		rightMaster.setInverted(false);
		// Set following
		leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    intakeMaster = new TalonSRX(6);
    intakeFollower = new TalonSRX(18);

    intakeMaster.setInverted(false);
    intakeMaster.setInverted(true);

    intakeFollower.follow(intakeMaster);
    
    intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 2);
    raisePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 3);

    elevatorMaster = new TalonSRX(9);
    elevatorFollowerA = new VictorSPX(7);
    elevatorFollowerB = new VictorSPX(8);
    elevatorFollowerC = new VictorSPX(10);

    elevatorFollowerA.follow(elevatorMaster);
    elevatorFollowerB.follow(elevatorMaster);
    elevatorFollowerC.follow(elevatorMaster);

    elevatorMaster.setNeutralMode(NeutralMode.Brake);
		elevatorFollowerA.setNeutralMode(NeutralMode.Brake);
		elevatorFollowerB.setNeutralMode(NeutralMode.Brake);
    elevatorFollowerC.setNeutralMode(NeutralMode.Brake);
    
    elevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    elevatorMaster.setSelectedSensorPosition(0);
    elevatorMaster.configForwardSoftLimitThreshold(18000);
    elevatorMaster.configReverseSoftLimitThreshold(1000);
    elevatorMaster.configReverseSoftLimitEnable(true);
    elevatorMaster.configForwardSoftLimitEnable(true);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("elevator pos", elevatorMaster.getSelectedSensorPosition());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double turn;
    if (controller.getRawButton(4)) {
      turn = 1.0;
    }
    else if (controller.getRawButton(3)) {
      turn = -1.0;
    }
    else {
      turn = 0.0;
    }

    double drive;
    if (controller.getRawButton(1)) {
      drive = 1.0;
    }
    else if (controller.getRawButton(2)) {
      drive = -1.0;
    }
    else {
      drive = 0.0;
    }

    double leftDrive = drive + turn * Math.abs(turn);
    double rightDrive = drive - turn * Math.abs(turn);
    leftDrive *= 0.4;
    rightDrive *= 0.4;

    leftMaster.set(ControlMode.PercentOutput, leftDrive);
    rightMaster.set(ControlMode.PercentOutput, rightDrive);
    
    double leftTrig;
    if (controller.getRawButton(10)) {
      leftTrig = 1.0;
    }
    else {
      leftTrig = 0.0;
    }

    double rightTrig;
    if (controller.getRawButton(9)) {
      rightTrig = 1.0;
    }
    else {
      rightTrig = 0.0;
    }

    if(leftTrig > rightTrig) {
      intakeMaster.set(ControlMode.PercentOutput, leftTrig * 0.4);
    } else {
      intakeMaster.set(ControlMode.PercentOutput, -rightTrig * 0.9);
    }
    
    if(controller.getRawButton(7)) {
      elevatorMaster.set(ControlMode.PercentOutput, 0.2, DemandType.ArbitraryFeedForward, 0.13);
    }

    if(controller.getRawButton(8)) {
      elevatorMaster.set(ControlMode.PercentOutput, -0.4, DemandType.ArbitraryFeedForward, 0.13);

    }
    if(operator.getRightBumperPressed()) {
      if(intakePiston.get() == DoubleSolenoid.Value.kOff)
        intakePiston.set(DoubleSolenoid.Value.kReverse);
      else intakePiston.toggle();
    }
    if(operator.getLeftBumperPressed()) {
      if(raisePiston.get() == DoubleSolenoid.Value.kOff)
        raisePiston.set(DoubleSolenoid.Value.kReverse);
      else raisePiston.toggle();
    }
    // if(controller.getPOV(0) < 45 && controller.getPOV(0) > 315)
    // {
    //   elevatorMaster.set(ControlMode.PercentOutput, 0.5);
    // }
    // else if (controller.getPOV(0) > 135 && controller.getPOV(0) < 225)
    // {
    //   elevatorMaster.set(ControlMode.PercentOutput, -0.5);
    // }
    // else intakeMaster.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
