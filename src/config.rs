#[derive(serde::Deserialize)]
/// Configuration for the simulation
pub struct Config {
    /// Simulation configuration
    pub simulation: SimulationConfig,
    /// Quadrotor configuration
    pub quadrotor: QuadrotorConfig,
    /// PID Controller configuration
    pub pid_controller: PIDControllerConfig,
}

#[derive(serde::Deserialize)]
/// Configuration for the simulation
pub struct SimulationConfig {
    /// Frequency of physics simulation updates (Hz)
    pub simulation_frequency: usize,
    /// Total duration of the simulation (seconds)
    pub duration: f32,
}

#[derive(serde::Deserialize)]
/// Configuration for the quadrotor
pub struct QuadrotorConfig {
    /// Mass of the quadrotor in kg
    pub mass: f32,
    /// Gravity in m/s^2
    pub gravity: f32,
    /// Drag coefficient in Ns^2/m^2
    pub drag_coefficient: f32,
    /// Inertia matrix in kg*m^2
    pub inertia_matrix: [f32; 9],
}

#[derive(serde::Deserialize)]
/// Configuration for the PID controller
pub struct PIDControllerConfig {
    /// Position gains
    pub pos_gains: PIDGains,
    /// Attitude gains
    pub att_gains: PIDGains,
    /// Maximum integral error for position control
    pub pos_max_int: [f32; 3],
    /// Maximum integral error for attitude control
    pub att_max_int: [f32; 3],
}

#[derive(serde::Deserialize)]
/// Configuration for PID gains
pub struct PIDGains {
    /// Proportional gains
    pub kp: [f32; 3],
    /// Integral gains
    pub ki: [f32; 3],
    /// Derivative gains
    pub kd: [f32; 3],
}
impl Config {
    /// Load configuration from a YAML file.
    /// # Arguments
    /// * `filename` - The name of the file to load.
    /// # Returns
    /// * The configuration object.
    /// # Errors
    /// * If the file cannot be read or the YAML cannot be parsed.
    pub fn from_yaml(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(filename)?;
        Ok(serde_yaml::from_str(&contents)?)
    }
}
