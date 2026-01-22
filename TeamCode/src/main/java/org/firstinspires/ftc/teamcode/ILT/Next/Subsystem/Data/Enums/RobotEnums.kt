package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums


/**
 * Controls how hood angle and flywheel speed are determined
 */
enum class OuttakeMode {
    IDLE,              // No automatic adjustment
    MANUAL,            // Operator controls hood/flywheel manually
    AUTO_ODOMETRY,     // Auto adjust using Pedro odometry distance
    AUTO_LIMELIGHT     // Auto adjust using Limelight distance
}

/**
 * Controls how the turret aims
 */
// change to turret mode once code finished
enum class TurretMode {
    IDLE,              // Turret stationary
    MANUAL,            // Direct joystick control
    LIMELIGHT,         // Track target using Limelight tx
    ODOMETRY_RELATIVE, // Aim at goal using motor encoder + odometry
    ODOMETRY_ABSOLUTE  // Aim at goal using absolute encoder + odometry
}

/**
 * Controls shooting behavior
 */
enum class ShootMode {
    IDLE,
    MANUAL,            // Driver triggers shots
    AUTO               // Automatic shooting when conditions met
}

/**
 * Intake state tracking
 */
enum class IntakeState {
    STOPPED,
    INTAKING,
    EJECTING,
    FEEDING
}

/**
 * Ball pattern detected by Limelight fiducials
 */
enum class Motif {
    GPP,   // Green Purple Purple
    PGP,   // Purple Green Purple
    PPG,   // Purple Purple Green
    NONE
}

/**
 * Ball color for sensor detection
 */
enum class BallColor {
    GREEN,
    PURPLE
}