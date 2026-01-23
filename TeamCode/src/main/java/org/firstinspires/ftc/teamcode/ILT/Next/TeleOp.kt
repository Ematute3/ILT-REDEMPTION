package org.firstinspires.ftc.teamcode.robot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.bindings.button
import dev.nextftc.control.KineticState
import dev.nextftc.core.commands.Command
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.ILT.Next.Commands.ShootCommands
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.Intake
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import org.firstinspires.ftc.teamcode.robot.data.enums.TurretMode
import org.firstinspires.ftc.teamcode.robot.data.enums.OuttakeMode
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.Gate
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.FlyWheel
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.OuttakeController
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.DriveTrain
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.Limelight
import kotlin.math.abs

@TeleOp(name = "Main TeleOp", group = "Competition")
class MainTeleOp : NextFTCOpMode() {

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                DriveTrain, FlyWheel, Intake, Turret,
                Hood, Limelight, OuttakeController, Gate
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        RobotConfig.alliance = Alliance.RED
        RobotState.reset()
    }

    override fun onStartButtonPressed() {
        // Pedro Pathing Driver Controlled Setup
        val driverControlled = PedroDriverControlled(
            { -gamepad1.left_stick_y.toDouble() },
            { -gamepad1.left_stick_x.toDouble() },
            { -gamepad1.right_stick_x.toDouble() },
            false // Field Centric
        )
        driverControlled.schedule()

        bindDriverControls()

    }

    private fun bindDriverControls() {
        // Intake Toggle
        button { gamepad1.left_trigger > 0.1f }
            .whenTrue(Intake.run)
            .whenFalse(Intake.stop)

        // Reverse Intake
        button { gamepad1.left_bumper }
            .whenTrue(Intake.run)
            .whenFalse(Intake.stop)
        button { gamepad1.right_trigger > 0.5f}
            .whenTrue(FlyWheel.spin)
        button{ gamepad1.right_bumper}
            .whenTrue { ShootCommands.shoot }
    }


    override fun onUpdate() {
        // 1. ==================== TURRET MANUAL OVERRIDE ====================

        val turretInput = gamepad2.right_stick_x.toDouble()
        val isManualNudging = abs(turretInput) > 0.1 || gamepad2.left_bumper || gamepad2.right_bumper

        if (isManualNudging) {
            RobotState.turretMode = TurretMode.MANUAL

            // Priority: Bumpers for fixed speed, Stick for variable speed
            Turret.manualPower = when {
                gamepad2.left_bumper -> -RobotConfig.TurretConfig.manualPowerFast
                gamepad2.right_bumper -> RobotConfig.TurretConfig.manualPowerFast
                else -> turretInput * RobotConfig.TurretConfig.manualPowerFast
            }
        }

        // 2. ==================== HOOD MANUAL ADJUST ====================
        val hoodInput = -gamepad2.left_stick_y.toDouble() // Negative for "Up is Up"
        if (abs(hoodInput) > 0.1) {
            // Manually nudging the hood position in RobotState
            RobotState.hoodPosition = (RobotState.hoodPosition + hoodInput * 0.005).coerceIn(0.0, 1.0)
        }

        // 3. ==================== TELEMETRY ====================
        telemetry.run {
            addData("Mode", "Outtake: ${RobotState.outtakeMode} | Turret: ${RobotState.turretMode}")
            addData("Target", if(RobotState.limelightHasTarget) "LOCKED" else "SEARCHING")
            addData("Ready", if(OuttakeController.canShoot()) "READY TO FIRE" else "WAITING...")
            addData("Dist (LL/Odom)", "%.1f / %.1f".format(RobotState.distanceToGoalLimelight ?: 0.0, RobotState.distanceToGoalOdometry))
            update()
        }
    }
}