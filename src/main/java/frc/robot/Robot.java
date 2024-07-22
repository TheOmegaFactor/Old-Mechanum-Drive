// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot 
{
  private final XboxController Ctrl = new XboxController(0);
  private final CANSparkMax FrontRight = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax FrontLeft = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax BackRight = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax BackLeft = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax WheelGoBrr = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax BaseLift1 = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax BaseLift2 = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax LauncherWrist1 = new CANSparkMax(8, MotorType.kBrushless);
  private final CANSparkMax LauncherWrist2 = new CANSparkMax(9, MotorType.kBrushless);
  private final CANSparkMax ClawLauncher1 = new CANSparkMax(10, MotorType.kBrushed);
  private final CANSparkMax ClawLauncher2 = new CANSparkMax(11, MotorType.kBrushed); 
  private final Compressor Cpm = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid Solo = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private final PIDController PIDCtrl = new PIDController(0.1, 0, 0);
  private final Timer timer = new Timer();
  private double Y_Pow;
  private double X_Pow;
  private double Rot_Pow;
  
  //knbopkoijf
  private final double MotorDiv = 1.75;


  @Override
  public void robotInit() 
  {
    FrontRight.setInverted(false);
    FrontLeft.setInverted(true);
    BackRight.setInverted(false);
    BackLeft.setInverted(true);
    WheelGoBrr.setInverted(true);
    BaseLift1.setInverted(true);
    BaseLift2.setInverted(true);
    LauncherWrist1.setInverted(true);
    LauncherWrist2.setInverted(false);
    ClawLauncher1.setInverted(true);
    ClawLauncher2.setInverted(false);

    Thread m_visionThread;
    m_visionThread = new Thread(
      () -> 
      {
        // Get the UsbCamera from CameraServer
        UsbCamera camera = CameraServer.startAutomaticCapture();
        // Set the resolution
        camera.setResolution(352, 288);
        camera.setFPS(20);
        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);
  
        Mat matt = new Mat();
  
        // This cannot be 'true'. The program will never exit if it is. This
        // lets the robot stop this thread when restarting robot code or
        // deploying.
        while (!Thread.interrupted()) 
        {
          // Tell the CvSink to grab a frame from the camera and put it
          // in the source mat.  If there is an error notify the output.
          if (cvSink.grabFrame(matt) == 0) 
          {
            // Send the output the error.
            outputStream.notifyError(cvSink.getError());
            // skip the rest of the current iteration
            continue;
          }
        }
      });
      m_visionThread.setDaemon(true);
      m_visionThread.start();
   }
  
  @Override
  public void robotPeriodic() 
  {

  }

  @Override
  public void autonomousInit() 
  {
    timer.reset();
    timer.start();
    LauncherWrist1.setInverted(true);
    LauncherWrist2.setInverted(false);
  }
  
  @Override
  public void autonomousPeriodic() 
  {
    LauncherWrist1.set(PIDCtrl.calculate(LauncherWrist1.getEncoder().getPosition(), Math.PI * 2.5));
    LauncherWrist2.set(PIDCtrl.calculate(LauncherWrist1.getEncoder().getPosition(), Math.PI * 2.5));

    if (timer.get() > 2 &&  timer.get() < 4)
    {
      ClawLauncher1.setInverted(false);
      ClawLauncher2.setInverted(true);
      ClawLauncher1.set(1);
      ClawLauncher2.set(1);
    }
    if (timer.get() > 4 && timer.get() < 6)
    {
      ClawLauncher1.set(0);
      ClawLauncher2.set(0);
      SetCtrl(0, -0.5, 0);
    }
    if (timer.get() > 6 && timer.get() < 7)
    {
      SetCtrl(0, 0, 0);
    }
    if (timer.get() > 7 && timer.get() < 9)
    {
      SetCtrl(0, 0, 0.3);
    }
    if (timer.get() > 9 && timer.get() < 10)
    {
      SetCtrl(0, 0, 0);
    }


  }

  public void SetCtrl(double XPower, double YPower, double RotPower)
  {
    Y_Pow = YPower;
    X_Pow = XPower;
    Rot_Pow = RotPower;

    FrontRight.set(Rot_Pow + Y_Pow + X_Pow);
    FrontLeft.set(-Rot_Pow + (Y_Pow - X_Pow));
    BackRight.set(Rot_Pow + (Y_Pow - X_Pow));
    BackLeft.set(-Rot_Pow + Y_Pow + X_Pow); 
  }

  public void StopMotors()
  {
    FrontRight.set(0);
    FrontLeft.set(0);
    BackLeft.set(0);
    BackRight.set(0);
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() 
  {
    if (Ctrl.getRawButton(1))
    {
      Y_Pow = Ctrl.getLeftY();
      X_Pow = Ctrl.getLeftX();
      Rot_Pow = Ctrl.getRightX();
    }
    else 
    {
      Y_Pow = Ctrl.getLeftY() / MotorDiv;
      X_Pow = Ctrl.getLeftX() / MotorDiv;
      Rot_Pow = Ctrl.getRightX() / MotorDiv;
    }
    
    FrontRight.set(Rot_Pow - Y_Pow - X_Pow);
    FrontLeft.set(-Rot_Pow + (-Y_Pow + X_Pow));
    BackRight.set(Rot_Pow + (-Y_Pow + X_Pow));
    BackLeft.set(-Rot_Pow - Y_Pow - X_Pow);
    WheelGoBrr.set(-Y_Pow);

    LauncherWrist1.set(Ctrl.getRightY() / 3);
    LauncherWrist2.set(Ctrl.getRightY() / 3);
    //PID Controller

    if (Ctrl.getLeftTriggerAxis() > 0.5)
    {
      ClawLauncher1.setInverted(true);
      ClawLauncher2.setInverted(false);
      ClawLauncher1.set(Ctrl.getLeftTriggerAxis() / 2);
      ClawLauncher2.set(Ctrl.getLeftTriggerAxis() / 2);
    }
    else if (Ctrl.getRightTriggerAxis() > 0.3)
    {
      ClawLauncher1.setInverted(false);
      ClawLauncher2.setInverted(true);
      ClawLauncher1.set(Ctrl.getRightTriggerAxis());
      ClawLauncher2.set(Ctrl.getRightTriggerAxis());
    }
    else 
    {
      ClawLauncher1.set(0);
      ClawLauncher2.set(0);
    }

    if(Ctrl.getRawButton(6))
    {
      Solo.set(Value.kForward);
      BaseLift1.set(0.5);
      BaseLift2.set(0.5);
    } 
    else if (Ctrl.getRawButton(5))
    {
      Solo.set(Value.kReverse);
      BaseLift1.set(-0.5);
      BaseLift2.set(-0.5);
    }
    else
    {
      BaseLift1.set(0);
      BaseLift2.set(0);
    }

    if (Ctrl.getRawButton(2))
    {
      LauncherWrist1.set(PIDCtrl.calculate(LauncherWrist1.getEncoder().getPosition(), Math.PI * 2.5));
      LauncherWrist2.set(PIDCtrl.calculate(LauncherWrist1.getEncoder().getPosition(), Math.PI * 2.5));
    }

    if (Ctrl.getRawButton(3))
    {
      LauncherWrist1.set(PIDCtrl.calculate(LauncherWrist1.getEncoder().getPosition(), Math.PI * 1.5));
      LauncherWrist2.set(PIDCtrl.calculate(LauncherWrist1.getEncoder().getPosition(), Math.PI * 1.5));
    }

    if (Ctrl.getRawButton(4))
    {
      LauncherWrist1.set(PIDCtrl.calculate(LauncherWrist1.getEncoder().getPosition(), Math.PI * 3.5));
      LauncherWrist2.set(PIDCtrl.calculate(LauncherWrist1.getEncoder().getPosition(), Math.PI * 3.5));
    }
    
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
  public void simulationPeriodic() {}
}
