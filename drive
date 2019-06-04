/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")

public class drive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armLeft = null;
    private DcMotor armRight = null;
    private DcMotor railLeft = null;
    private DcMotor railRight = null;
    private CRServo claw = null;
    private CRServo lock = null;
    
    static final double MOTOR_SLOW = .275; //ratio of speed for slow mode for motors

    public static boolean floatToBool(float f){return f>0;}

    @Override
    public void runOpMode() {
        

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        railLeft = hardwareMap.get(DcMotor.class, "railLeft");
        railRight = hardwareMap.get(DcMotor.class, "railRight");
        claw = hardwareMap.get(CRServo.class, "claw");
        lock = hardwareMap.get(CRServo.class, "lock");
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        armLeft.setDirection(DcMotor.Direction.REVERSE);
        armRight.setDirection(DcMotor.Direction.FORWARD);
        railLeft.setDirection(DcMotor.Direction.REVERSE);
        railRight.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(DcMotor.Direction.REVERSE);
        lock.setDirection(DcMotor.Direction.REVERSE);
        
        telemetry.addData("Status", "Driver mode ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            //raises rail while pressing a, lowers rail while pressing b
            double rail_power =0;
            if(gamepad1.a)
            {
                rail_power = 1/2.0;
                railLeft.setPower(rail_power);
                railRight.setPower(rail_power);
            }
            else if(gamepad1.b)
            {
                rail_power = -1/2.0;
                railLeft.setPower(rail_power);
                railRight.setPower(rail_power);
            }
            else
            {
                rail_power=0;
                railLeft.setPower(rail_power);
                railRight.setPower(rail_power);
            }
            
            //holding left trigger opens lock and holding right trigger closses it
            if(gamepad2.left_bumper)
            {
                lock.setPower(1);
            }
            else if(gamepad2.right_bumper)
            {
                lock.setPower(-1);
            }
            else
            {
                lock.setPower(0);
            }
            
            //holding a closes the claw, holding b opens the claw
            if(gamepad2.a)
            {
                claw.setPower(1);
            }
            else if(gamepad2.b)
            {
                claw.setPower(-1);
            }
            else
            {
                claw.setPower(0);
            }
            
            //Right stick for controller 2 controls arm. Holding x makes it stop moving.
            double arm =gamepad2.right_stick_y;
            if(gamepad2.x)
            {
                armLeft.setPower(-.06);
                armRight.setPower(-.06);
            }
            else
            {
                armLeft.setPower(arm/3);
                armRight.setPower(arm/3);
            }
            
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // POV Mode uses left stick to go forward, and right stick to turn.
            double drive = gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;
            if(drive==0)
            {
                //turning is slowed while stationary
                leftPower    = Range.clip(drive - turn/2, -1.0, 1.0);
                rightPower   = Range.clip(drive + turn/2, -1.0, 1.0);
            }
            else
            {
                //turning is faster while driving forwards/backwards
                leftPower    = Range.clip(drive - turn, -1.0, 1.0);
                rightPower   = Range.clip(drive + turn, -1.0, 1.0);
            }
            
            //hold left trigger for slow mode
            if(floatToBool(gamepad1.left_trigger))
            {
                leftDrive.setPower(leftPower*MOTOR_SLOW);
                rightDrive.setPower(rightPower*MOTOR_SLOW);
            }
            //normal controls
            else
            {
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);   
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), arm (%.2f)", leftPower, rightPower,arm);
            telemetry.update();
        }
    }
}
