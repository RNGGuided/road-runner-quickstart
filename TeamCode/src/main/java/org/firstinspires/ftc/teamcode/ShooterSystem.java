package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;



public class ShooterSystem {


    private DcMotor shooterLeft, shooterRight, intake;
    private CRServo feederServo, spindexerServo, feederServo2;


    private Servo Kicker1,Kicker2;


    private AnalogInput AnalogEncoder, servoEncoder;


    public double optimalAngle1 = 0, optimalAngle2 = 0, optimalAngle3 = 0;


    public double[] optimalAnglesSERVO = new double[3];




    public ShooterSystem(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");//port 2
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");// port 3
        intake = hardwareMap.get(DcMotor.class, "intake");//port0
        feederServo = hardwareMap.get(CRServo.class, "feederServo");// servo port 4
        Kicker1 = hardwareMap.get(Servo.class, "Kicker1");// servo port 1
        Kicker2 =hardwareMap.get(Servo.class, "Kicker2");// servo port 3
        spindexerServo = hardwareMap.get(CRServo.class, "SpindexerServo");// servo port 2
        AnalogEncoder = hardwareMap.get(AnalogInput.class, "MelonEncoder1");
        feederServo2 = hardwareMap.get(CRServo.class, "feederServo2");// 0
        servoEncoder = hardwareMap.get(AnalogInput.class, "servoEncoder");
        //shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER)


    }


    // Function to spin the two shooter motors
    public void shoot(double power) {
        shooterLeft.setPower(-power);
        shooterRight.setPower(-power);
        //intake.setPower(-power);
    }


    // Function to stop the shooter
    public void stopShooter() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        //intake.setPower(0);
    }


    // Function to on the intake
    public void intakeOn (double power) {
        intake.setPower(-power);
    }


    // Function to off the intake
    public void intakeOff () {
        intake.setPower(0.0);
    }


    public void intakeReverse (double power)
    {
        intake.setPower(power);
    }


    public void KickerUp() {
        Kicker1.setPosition(0);
        Kicker2.setPosition(.5);
    }
    public void KickerDown() {
        Kicker1.setPosition(.5);
        Kicker2.setPosition(0);
    }


    // Function to control continuous rotation servo
    // rightTrigger -> forward, leftTrigger -> reverse
    public void controlFeeder(double rightTrigger, double leftTrigger) {
        double servoPower = rightTrigger - leftTrigger; // combine both triggers
        feederServo.setPower(servoPower);
        feederServo2.setPower(-servoPower);
    }
   /*public void controlSpindexer(double angleS)
   {
       final double optimalAngle1 = 16.1455;
       final double optimalAngle2 = 136.1455;
       final double optimalAngle3 = 256.1455;
       double angle;
       boolean skipOA1 = false;
       boolean skipOA2 = false;
       boolean skipOA3 = false;




       if((angleS > optimalAngle1 - 5) && (angleS < optimalAngle1 + 5))
       {
           skipOA1 = true;
       }
       if((angleS > optimalAngle2 - 5) && (angleS < optimalAngle2 + 5))
       {
           skipOA2 = true;
       }
       if((angleS > optimalAngle3 - 5) && (angleS < optimalAngle3 + 5))
       {
           skipOA3 = true;
       }


       while (true)
       {
           double voltage = AnalogEncoder.getVoltage(); // get voltage (0V–3.3V)
           angle = (voltage / 3.3) * 360.0;
           if( (angle > optimalAngle1 - 5) && (angle < optimalAngle1 + 5) && !skipOA1)
           {
               while( (angle > optimalAngle1 + 1) )
               {
                   spindexerServo.setPower(-0.5);
               }


               spindexerServo.setPower(0);
               break;
           }
           if( (angle > optimalAngle2 - 5) && (angle < optimalAngle2 + 5) && !skipOA2)
           {
               while( (angle > optimalAngle2 + 1) )
               {
                   spindexerServo.setPower(-0.5);
               }


               spindexerServo.setPower(0);
               break;
           }
           if( (angle > optimalAngle3 - 5) && (angle < optimalAngle3 + 5) && !skipOA3)
           {
               while( (angle > optimalAngle1 + 1) || angle < 15)
               {
                   spindexerServo.setPower(-0.5);
               }


               spindexerServo.setPower(0);
               break;
           }


           spindexerServo.setPower(0.1);
       }


   }
   /*
   public void stopSpindexer()
   {
       spindexerServo.setPower(0);
   }
   */




    boolean spinning = false;
    double targetAngle = 0;
    final double TOLERANCE = 5;


    // 3 optimal positions (degrees)
    final double[] optimalAngles = {13.1455+8.5, 255.7091+8.5,137.1455+8.5};
    int currentTargetIndex = -1;  // start with none selected


    public void spinToNextAngle(double currentAngle) {
        if (spinning) return;


        // Find next target in sequence
        if (currentTargetIndex == -1) {
            currentTargetIndex = 0; // start at first angle
        } else {
            currentTargetIndex = (currentTargetIndex + 1) % optimalAngles.length;
        }


        targetAngle = optimalAngles[currentTargetIndex];
        spinning = true;


        // Always rotate in one direction (clockwise)
        spindexerServo.setPower(0.1);
    }


    public void update() {
        if (!spinning) return;


        double voltage = AnalogEncoder.getVoltage();
        double angle = (voltage / 3.3) * 360.0;


        // Calculate smallest CW distance between current and target
        double cwDistance = (targetAngle - angle + 360) % 360;


        // Stop when within tolerance (close enough in CW direction)
        if (cwDistance < TOLERANCE) {
            spindexerServo.setPower(0);
            spinning = false;
        }
    }


    public boolean isSpinning() {
        return spinning;
    }


    int initCycles = 0;
    boolean optimalSet = false;


    public void getOptimalAngles() {
        if (optimalSet) return;  // only run once


        double voltage2 = servoEncoder.getVoltage();
        double angle2 = (voltage2 / 3.3) * 360.0;


        if (initCycles < 5) {  // wait ~25 loop cycles (about 0.5s)
            initCycles++;
            return;
        }


        // After waiting:
        optimalAngle1 = angle2;
        optimalAngle2 = (angle2 + 40) % 360;
        optimalAngle3 = (angle2 + 240) % 360;
        optimalSet = true;
        optimalAnglesSERVO[0] = (optimalAngle1+3)%360;
        optimalAnglesSERVO[1] = optimalAngle1;
        optimalAnglesSERVO[2] = (optimalAngle1-3)%360;
    }
    boolean spinningServo = false;
    double targetAngleServo = 0;
    final double TOLERANCESERVO = 2; // smaller tolerance since PID is smoother


    // 3 optimal positions (degrees)
    int currentTargetIndexServo = -1;
    int directionServo = 1;


    // PID constants (tune these!)
    double kP = 0.08;  // proportional gain
    double kI = 0.000; // integral gain
    double kD = 0.001;  // derivative gain


    // PID state variables
    double integralServo = 0;
    double lastErrorServo = 0;
    long lastTimeServo = 0;


    public void spinToNextAngleServo() {
        if (spinningServo) return;


        // Pick next target angle
        if (currentTargetIndexServo == -1) {
            currentTargetIndexServo = 0;
        } else {
            currentTargetIndexServo += directionServo;


            // Reverse direction when reaching ends
            if (currentTargetIndexServo >= optimalAnglesSERVO.length) {
                currentTargetIndexServo = optimalAnglesSERVO.length - 2;
                directionServo = -1;
            } else if (currentTargetIndexServo < 0) {
                currentTargetIndexServo = 1;
                directionServo = 1;
            }
        }


        targetAngleServo = optimalAnglesSERVO[currentTargetIndexServo];
        spinningServo = true;


        // Reset PID state
        integralServo = 0;
        lastErrorServo = 0;
        lastTimeServo = System.currentTimeMillis();
    }


    public void updateServo() {
        if (!spinningServo) return;


        double voltage2 = servoEncoder.getVoltage();
        double angle2 = (voltage2 / 3.3) * 360.0;


        // Compute error (normalize to -180 to +180)
        double error = targetAngleServo - angle2;
        error = ((error + 540) % 360) - 180;


        // Time difference for derivative term
        long now = System.currentTimeMillis();
        double deltaTime = (now - lastTimeServo) / 1000.0;
        if (deltaTime <= 0) deltaTime = 0.001; // avoid division by zero


        // PID calculations
        integralServo += error * deltaTime;
        double derivative = (error - lastErrorServo) / deltaTime;


        double power = kP * error + kI * integralServo + kD * derivative;


        // Clamp power
        power = Math.max(-0.3, Math.min(0.3, power));


        // Apply power (reverse one servo)
        feederServo.setPower(power);
        feederServo2.setPower(-power);


        // Save for next loop
        lastErrorServo = error;
        lastTimeServo = now;


        // Stop when within tolerance and moving slowly
        if (Math.abs(error) < TOLERANCESERVO && Math.abs(power) < 0.02) {
            feederServo.setPower(0);
            feederServo2.setPower(0);
            spinningServo = false;
        }
    }


    public boolean isSpinningServo() {
        return spinningServo;
    }
}

