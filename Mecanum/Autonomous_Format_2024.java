package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import java.util.List;

@Autonomous(name = "Autonomous_Format_2024", group = "")
//@Disabled
public class Autonomous_Format_2024 extends LinearOpMode {

  BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;
  private DcMotor FrontRight;
  private DcMotor FrontLeft;
  private DcMotor RearRight;
  private DcMotor RearLeft;

  int Ticks = 0;
  int Ticks2 = 0;
  int Distance2;
  double Speed = 0.65;

//  double TICKS_PER_INCH = 39.76;
  double TICKS_PER_INCH = 42;



  @Override
  public void runOpMode() {




    //Put Motor Name = dcMotor Motor Name here 
   FrontRight=hardwareMap.dcMotor.get("rightFront");
   FrontLeft=hardwareMap.dcMotor.get("leftFront");
   RearRight=hardwareMap.dcMotor.get("rightRear");
   RearLeft=hardwareMap.dcMotor.get("leftRear");

  FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
  RearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
  FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);


  FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

  BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
      parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
      parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
      parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
      parameters.loggingEnabled      = true;
      parameters.loggingTag          = "IMU";
      parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

  imu = hardwareMap.get(BNO055IMU.class, "imu");
  imu.initialize(parameters);


    telemetry.addData(">", "Press Play to start op mode");
    telemetry.update(); 
    waitForStart();
    if (opModeIsActive()) {



        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      telemetry.addData("Ticks:",Ticks);
      telemetry.update();
      telemetry.addData("Ticks2:",Ticks2);
      telemetry.update();
      //////////////////////////////////////////////////////////////////////////////////
      //        Create list of Commands for the Auto here
      //////////////////////////////////////////////////////////////////////////////////
      Speed = 0.35;

      //Drive(27.5,Speed); 
      Drive_Simple(3000,0.5);
      sleep(3000);
      Turn(-45,1);
      Drive(5,Speed);

      //////////////////////////////////////////////////////////////////////////////////
    }
  }

   private void initializeGyro() {
          BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
          parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
          parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
          parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
          parameters.loggingEnabled      = true;
          parameters.loggingTag          = "IMU";        
          parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            telemetry.addLine("Gyro Calibrating...");
            telemetry.update();
        } 
   }


    private void Turn(int Angle, double Power) 
{     
      //TURN RIGHT IS POSITIVE
      //TURN LEFT IS NEGATIVE

      telemetry.addLine("We are now inside Turn function");
      telemetry.update();

      angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      float RobotAngle = -angles.firstAngle;
      float StartingAngle = RobotAngle;
      boolean StartFast = false;
      telemetry.addData("Gyro angle", RobotAngle);
      telemetry.update();

      FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      float TotalAngle = Math.abs(RobotAngle-Angle);
      float StartSlowingDownAngleOffset = 0;

      if (TotalAngle < 40){        
        StartFast = false;
      }
      else if (TotalAngle > 90)
      {
        StartSlowingDownAngleOffset = (TotalAngle-90)/9;
      }
      else
      {      
        StartFast = true;
      }

      if (Angle < RobotAngle) {
        while ((Angle < RobotAngle - (75+StartSlowingDownAngleOffset)) || (((StartingAngle - RobotAngle)  < 1) && StartFast))
          {
            telemetry.addData("Gyro",RobotAngle );
            telemetry.addData("TotalAngle",TotalAngle );
            telemetry.addData("StartSlowingDownAngleOffset",StartSlowingDownAngleOffset );
            telemetry.update();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            RobotAngle = -angles.firstAngle;
            FrontLeft.setPower(-Power);
            FrontRight.setPower(Power);
            RearRight.setPower(Power);
            RearLeft.setPower(-Power);
          }
        while (Angle < RobotAngle-2.5)
          {
            telemetry.addData("Gyro",RobotAngle );
            telemetry.addData("TotalAngle",TotalAngle );
            telemetry.addData("StartSlowingDownAngleOffset",StartSlowingDownAngleOffset );
            telemetry.update();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            RobotAngle = -angles.firstAngle;
            FrontLeft.setPower(-0.25);
            FrontRight.setPower(0.25);
            RearRight.setPower(0.25);
            RearLeft.setPower(-0.25);
          }



          FrontLeft.setPower(0);
          FrontRight.setPower(0);
          RearRight.setPower(0);
          RearLeft.setPower(0);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        RobotAngle = -angles.firstAngle;


      }



       else {

          while ((Angle - (75+StartSlowingDownAngleOffset) > RobotAngle) || (((RobotAngle-StartingAngle)  < 1) && StartFast))
          {
            telemetry.addData("Gyro",RobotAngle );
            telemetry.addData("TotalAngle",TotalAngle );
            telemetry.addData("StartSlowingDownAngleOffset",StartSlowingDownAngleOffset );
            telemetry.update();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            RobotAngle = -angles.firstAngle;
            FrontLeft.setPower(Power);
            FrontRight.setPower(-Power);
            RearRight.setPower(-Power);
            RearLeft.setPower(Power);
          }
          FrontLeft.setPower(0);
          FrontRight.setPower(0);
          RearRight.setPower(0);
          RearLeft.setPower(0);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

          while (Angle - 2.5 > RobotAngle)
          {
            telemetry.addData("Gyro",RobotAngle );
            telemetry.addData("TotalAngle",TotalAngle );
            telemetry.addData("StartSlowingDownAngleOffset",StartSlowingDownAngleOffset );
            telemetry.update();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            RobotAngle = -angles.firstAngle;
            FrontLeft.setPower(0.25);
            FrontRight.setPower(-0.25);
            RearRight.setPower(-0.25);
            RearLeft.setPower(0.25);
          }
          FrontLeft.setPower(0);
          FrontRight.setPower(0);
          RearRight.setPower(0);
          RearLeft.setPower(0);
       }


}


    private void Drive(double Distance, double Power) {
      //Distance2 = Distance * 50;
 //     Ticks = Distance2 - 50;
      Ticks = (int) (TICKS_PER_INCH * Distance);

      //sleep(100);


      FrontRight.setTargetPosition(FrontRight.getCurrentPosition() + Ticks);
      FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + Ticks);
      RearRight.setTargetPosition(RearRight.getCurrentPosition() + Ticks);
      RearLeft.setTargetPosition(RearLeft.getCurrentPosition() + Ticks);
      FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontRight.setPower(Power);
      FrontLeft.setPower(Power);
      RearRight.setPower(Power);
      RearLeft.setPower(Power);
      while(FrontRight.isBusy())
      {
        telemetry.addData("CPosition",FrontRight.getCurrentPosition() );
        telemetry.update();
        if(isStopRequested() || !opModeIsActive()){
          break;
        }
      }
      FrontRight.setPower(0);
      FrontLeft.setPower(0);
      RearRight.setPower(0);
      RearLeft.setPower(0);

      sleep(100);
      FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

  private void Drive_Simple(int Time, double Power) {
    
    FrontRight.setPower(Power);
    FrontLeft.setPower(Power);
    RearRight.setPower(Power);
    RearLeft.setPower(Power);

    sleep(Time);
    
    FrontRight.setPower(0);
    FrontLeft.setPower(0);
    RearRight.setPower(0);
    RearLeft.setPower(0);

    


  }
}




