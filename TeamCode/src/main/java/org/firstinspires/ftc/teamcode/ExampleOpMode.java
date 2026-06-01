package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



import com.cartographer.cartographer.Angle;
import com.cartographer.cartographer.AngleType;
import com.cartographer.cartographer.ControllerType;
import com.cartographer.cartographer.DrivetrainType;
import com.cartographer.cartographer.Point;
import com.cartographer.cartographer.RotationType;
import com.cartographer.cartographer.controller.SquIDFController;
import com.cartographer.cartographer.drivetrain.MecanumDrivetrain;
import com.cartographer.cartographer.events.FunctionEvent;
import com.cartographer.cartographer.events.GotoEvent;
import com.cartographer.cartographer.events.SleepEvent;
import com.cartographer.cartographer.helpers.Encoder;
import com.cartographer.cartographer.helpers.EncoderDirection;
import com.cartographer.cartographer.odometry.TwoWheelIMUOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.cartographer.cartographer.Mapper;


@Autonomous(name="Example: Cartographer", group="Linear OpMode")
public class ExampleOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backRightDrive = null;

    private Mapper cartographer;

    private MecanumDrivetrain drivetrain = new MecanumDrivetrain();

    private TwoWheelIMUOdometry odometry = new TwoWheelIMUOdometry();

    private Point scorePoint = new Point(40, 72);
    private Point homePoint = new Point(128, 72);

    void launchBall(){
        // example function
    }

    void spinupFlywheel(){
        // example function
    }


    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain();

        // Setup Mecanum info, it's important you use DcMotorEx, especially if you are using motor encoders
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fr");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "br");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        drivetrain.SetMotors(
                (frontLeftDrive),
                (frontRightDrive),
                (backLeftDrive),
                (backRightDrive));

        odometry.SetDrivePod(new Encoder(frontLeftDrive, EncoderDirection.FORWARDS, 4));
        odometry.SetStrafePod(new Encoder(frontRightDrive, EncoderDirection.FORWARDS, 4));

        /* If you have a different drivetrain than Mecanum or Differential:
            1. Choose DrivetrainType.CUSTOM
            2. Create a new Java class that implements cartographer.drivetrain.IDrivetrain
            3. Implement the Update and Drive methods (see MecanumDrivetrain.java)
            4. Define the capabilities section
            5. Pass it through to Init, since IDrivetrain is an interface it should be plug and play.
         */

        cartographer = new Mapper.Builder()
                .drivetrain(DrivetrainType.MECANUM, drivetrain)
                .odometry(odometry)
                .driveController(new SquIDFController(0.1, 0)) // controllers are not enforced but HIGHLY recommended. SquIDF is the only one provided by default, however writing your own isn't hard
                .headingController(new SquIDFController(0.1, 0))
                .translationalController(new SquIDFController(0.1, 0))
                .build();

        cartographer.SetPower(1.0);

        // new Angle(270) will automatically be in degrees, specifying degrees is not necessary, you only need to specify radians
        // this is just to show you how it works
        cartographer.SetLocation(new Point(0, 0, new Angle(270, AngleType.DEGREES), RotationType.ROTATE_THEN_MOVE));
        // if you do NOT set rotation type, the rotation will be ignored and your robot will either:
        // 1. Maintain it's current rotation (for holonomic drives like Mecanum)
        // 2. Stay in their tangential direction (for tank drives like Differential)


        // You can reuse points but DO NOT REUSE EVENTS!!!
        /*cartographer.eventQueue.Enqueue(new GotoEvent(scorePoint));

        cartographer.eventQueue.Enqueue(new FunctionEvent(this::spinupFlywheel));

        cartographer.eventQueue.Enqueue(new SleepEvent(5));

        cartographer.eventQueue.Enqueue(new FunctionEvent(this::launchBall));

        cartographer.eventQueue.Enqueue(new SleepEvent(2));

        cartographer.eventQueue.Enqueue(new GotoEvent(homePoint));*/


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            cartographer.Update();

            telemetry.addData("Distance", String.valueOf(cartographer.GetDistanceToTarget()));
            telemetry.addData("State ID", String.valueOf(cartographer.eventQueue.GetStateID()));

            telemetry.addData("X", String.valueOf(cartographer.GetLocation().x));
            telemetry.addData("Y", String.valueOf(cartographer.GetLocation().y));
            telemetry.addData("Heading", String.valueOf(cartographer.GetLocation().heading));

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());


            telemetry.update();
        }
    }}

