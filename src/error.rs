#[derive(thiserror::Error, Debug)]
pub enum SimulationError {
    /// Error related to Rerun visualization
    #[error("Rerun error: {0}")]
    RerunError(#[from] rerun::RecordingStreamError),
    /// Error related to Rerun spawn process
    #[error("Rerun spawn error: {0}")]
    RerunSpawnError(#[from] rerun::SpawnError),
    /// Error related to Rerun logger setup
    #[error("Rerun SetLogger error: {0}")]
    SetLoggerError(#[from] rerun::external::log::SetLoggerError),
    /// Error related to linear algebra operations
    #[error("Nalgebra error: {0}")]
    NalgebraError(String),
    /// Error related to the OSQP solver
    /// Other general errors
    #[error("Other error: {0}")]
    OtherError(String),
}

