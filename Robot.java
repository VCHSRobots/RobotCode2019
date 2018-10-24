/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4415.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot {
	Button button1;
	Button button2; 
	Joystick controller; 
	WPI_TalonSRX m_jig;
	Encoder coder;
	Button button3;
	Button button4;
	boolean firsttime = true;
	DigitalInput limitswitch;
	WPI_TalonSRX m_jig2;
	
	@Override
	public void robotInit() {
		controller = new Joystick(0);
		m_jig = new WPI_TalonSRX(19);
		button1 = new JoystickButton(controller, 1);
		button2 = new JoystickButton(controller, 2);
		button3 = new JoystickButton(controller, 3);
		button4 = new JoystickButton(controller, 4);
		m_jig.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		m_jig.configOpenloopRamp(0.5, 0);
		m_jig.configClosedloopRamp(0, 0);
		m_jig.setSensorPhase(true);
		limitswitch = new DigitalInput(19);
		m_jig2 = new WPI_TalonSRX(22);
	}
		
	public void robotPeriodic( ) {
		SmartDashboard.putNumber("Current", m_jig.getOutputCurrent());
		SmartDashboard.putNumber("Velocity", m_jig.getSelectedSensorVelocity());
		SmartDashboard.putNumber("Joy-Y",controller.getY());
		SmartDashboard.putNumber("Position", m_jig.getSelectedSensorPosition());
		double rpm = m_jig.getSelectedSensorVelocity() * 10 * 60 / 1024;
		SmartDashboard.putNumber("rpm",rpm);
	}
	
	
	@Override
	public void teleopPeriodic() {
	
		m_jig.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0); // PIDLoop=0, timeoutMs=0
		
		m_jig.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, 0);
		m_jig.configVelocityMeasurementWindow(64, 0); 
		
		int quadPOS = m_jig.getSensorCollection().getQuadraturePosition();
		SmartDashboard.putNumber("Quadratic Position", quadPOS);
	
		if (button2.get() == true) {
			m_jig.set(ControlMode.PercentOutput,.3);
		} else {
			m_jig.set(ControlMode.PercentOutput,0.0);
		}
		
		if (button3.get() == true) {
			if (firsttime) {
				double iaccum = 0;
				m_jig.setIntegralAccumulator(iaccum, 0, 10);
				firsttime = false;
			}
			m_jig.set(ControlMode.Velocity, 1500 * 1024 / 600);
		} else if (button4.get() == true) {
			if (firsttime) {
				double iaccum = 0;
				m_jig.setIntegralAccumulator(iaccum, 0, 10);
				firsttime = false;
			}
			m_jig.set(ControlMode.Velocity, -1500 * 1024 / 600);
		} else { 
			m_jig.set(ControlMode.Velocity,0);
			firsttime = true;
		}
		m_jig2.set(ControlMode.Follower, 19);
		DoubleSolenoid exampleDouble = new DoubleSolenoid(1, 2);

		exampleDouble.set(DoubleSolenoid.Value.kOff);
		exampleDouble.set(DoubleSolenoid.Value.kForward);
		exampleDouble.set(DoubleSolenoid.Value.kReverse);	
	}
}
	
		 /* double rightstick; 
		rightstick = controller.getY() / 2;
		SmartDashboard.putNumber("PercentOutput", rightstick);
		m_jig.set(ControlMode.PercentOutput, rightstick);
		SmartDashboard.putNumber("Current", rightstick); */


