package org.firstinspires.ftc.teamcode.robot.subsystems.shooter

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot.AimbotTable
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.OuttakeMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.TurretMode

import org.firstinspires.ftc.teamcode.robot.data.config.RobotState


/**
 * Unified outtake controller.
 * Coordinates FlyWheel, Hood, and Turret based on current modes.
 *
 * This subsystem does NOT own the hardware - it just coordinates the other subsystems.
 * Add FlyWheel, Hood, and Turret as separate subsystems.
 */
object OuttakeController : Subsystem {

    // Manual aim distance (for MANUAL mode)
    var manualAimDistance = 48
        private set

    // Track previous modes to detect changes
    private var prevOuttakeMode: OuttakeMode? = null
    private var prevTurretMode: TurretMode? = null

    override fun periodic() {
        // Detect mode changes
        val outtakeModeChanged = RobotState.outtakeMode != prevOuttakeMode
        val turretModeChanged = RobotState.turretMode != prevTurretMode

        // Handle outtake modes (hood + flywheel)
        when (RobotState.outtakeMode) {
            OuttakeMode.IDLE -> {
                if (outtakeModeChanged) {
                    RobotState.hoodPosition = 0.0
                    RobotState.flywheelOn = false
                }
            }

            OuttakeMode.MANUAL -> {
                applyManualAim()
            }

            OuttakeMode.AUTO_ODOMETRY -> {
                if (RobotState.poseValid) {
                    applyAutoAimOdometry()
                } else {
                    ActiveOpMode.telemetry.addData("Auto Aim", "Waiting for pose...")
                }
            }

            OuttakeMode.AUTO_LIMELIGHT -> {
                if (RobotState.limelightHasTarget) {
                    applyAutoAimLimelight()
                } else {
                    ActiveOpMode.telemetry.addData("LL Aim", "No target")
                }
            }
        }

        // Handle turret modes
        when (RobotState.turretMode) {
            TurretMode.IDLE -> {
                Turret.setManualPower(0.0)
            }

            TurretMode.MANUAL -> {
                Turret.setManualPower(Turret.manualPower)
            }

            TurretMode.LIMELIGHT -> {
                if (RobotState.limelightReady && RobotState.limelightHasTarget) {
                    Turret.autoAimLimelight()
                } else {
                    Turret.setManualPower(0.0)
                }
            }

            TurretMode.ODOMETRY_RELATIVE -> {
                if (RobotState.poseValid) {
                    Turret.autoAim(useAbsolute = false)
                } else {
                    Turret.setManualPower(0.0)
                }
            }

            TurretMode.ODOMETRY_ABSOLUTE -> {
                if (RobotState.poseValid) {
                    Turret.autoAim(useAbsolute = true)
                } else {
                    Turret.setManualPower(0.0)
                }
            }
        }

        // Update previous modes
        prevOuttakeMode = RobotState.outtakeMode
        prevTurretMode = RobotState.turretMode

        // Telemetry
        ActiveOpMode.telemetry.run {
            addData("=== OUTTAKE ===", "")
            addData("Outtake Mode", RobotState.outtakeMode)
            addData("Turret Mode", RobotState.turretMode)
            addData("Manual Distance", manualAimDistance)
        }
    }

    // ==================== AIM LOGIC ====================

    private fun applyManualAim() {
        val snapped = AimbotTable.snapToValidDistance(manualAimDistance)
        val values = AimbotTable.getValues(snapped.toDouble())

        if (values != null) {
            RobotState.hoodPosition = values.first
            FlyWheel.setTargetVelocity(values.second)
        }
    }

    private fun applyAutoAimOdometry() {
        val distance = RobotState.distanceToGoalOdometry
        val values = AimbotTable.getValues(distance)

        if (values != null) {
            // Apply with offsets
            RobotState.hoodPosition = (values.first + 0.06).coerceIn(0.0, 1.0)
            FlyWheel.setTargetVelocity(values.second + 100)
        }
    }

    private fun applyAutoAimLimelight() {
        val distance = RobotState.distanceToGoalLimelight ?: return
        val values = AimbotTable.getValues(distance)

        if (values != null) {
            RobotState.hoodPosition = (values.first + 0.06).coerceIn(0.0, 1.0)
            FlyWheel.setTargetVelocity(values.second + 100)
        }
    }

    // ==================== MANUAL AIM ADJUSTMENT ====================

    fun increaseManualDistance() {
        manualAimDistance = (manualAimDistance + 12).coerceAtMost(144)
    }

    fun decreaseManualDistance() {
        manualAimDistance = (manualAimDistance - 12).coerceAtLeast(12)
    }

    fun setManualDistance(distance: Int) {
        manualAimDistance = AimbotTable.snapToValidDistance(distance)
    }

    // ==================== MODE COMMANDS ====================

    val idleMode = InstantCommand {
        RobotState.outtakeMode = OuttakeMode.IDLE
        RobotState.turretMode = TurretMode.IDLE
    }

    val manualMode = InstantCommand {
        RobotState.outtakeMode = OuttakeMode.MANUAL
        RobotState.turretMode = TurretMode.MANUAL
    }

    val autoOdometryMode = InstantCommand {
        if (!RobotState.poseValid) {
            ActiveOpMode.telemetry.addData("Mode", "Cannot use auto - no pose")
            return@InstantCommand
        }
        RobotState.outtakeMode = OuttakeMode.AUTO_ODOMETRY
        RobotState.turretMode = TurretMode.ODOMETRY_ABSOLUTE
    }

    val autoLimelightMode = InstantCommand {
        if (!RobotState.limelightReady) {
            ActiveOpMode.telemetry.addData("Mode", "Cannot use LL - not ready")
            return@InstantCommand
        }
        RobotState.outtakeMode = OuttakeMode.AUTO_LIMELIGHT
        RobotState.turretMode = TurretMode.LIMELIGHT
    }

    val aimUp = InstantCommand { increaseManualDistance() }
    val aimDown = InstantCommand { decreaseManualDistance() }
}