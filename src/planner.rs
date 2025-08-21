use nalgebra::{
    Vector3,
};
use crate::error::SimulationError;
pub trait Planner {
    /// Plans the trajectory based on the current state and time
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `current_velocity` - The current velocity of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// * A tuple containing the desired position, velocity, and yaw angle
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::{Planner, SimulationError};
    /// struct TestPlanner;
    /// impl Planner for TestPlanner {
    ///     fn plan(
    ///         &self,
    ///         current_position: Vector3<f32>,
    ///         current_velocity: Vector3<f32>,
    ///         time: f32,
    /// ) -> (Vector3<f32>, Vector3<f32>, f32) {
    ///         (Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 0.0)
    ///     }
    ///     fn is_finished(
    ///         &self,
    ///         current_position: Vector3<f32>,
    ///         time: f32,
    ///     ) -> Result<bool, SimulationError> {
    ///         Ok(true)
    ///     }
    /// }
    /// ```
    fn plan(
        &self,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32);
    /// Checks if the current trajectory is finished
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// * `true` if the trajectory is finished, `false` otherwise
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::{Planner, SimulationError};
    /// struct TestPlanner;
    /// impl Planner for TestPlanner {
    ///     fn plan(
    ///         &self,
    ///         current_position: Vector3<f32>,
    ///         current_velocity: Vector3<f32>,
    ///         time: f32,
    /// ) -> (Vector3<f32>, Vector3<f32>, f32) {
    ///         (Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 0.0)
    ///     }
    ///     fn is_finished(
    ///         &self,
    ///         current_position: Vector3<f32>,
    ///         time: f32,
    ///     ) -> Result<bool, SimulationError> {
    ///         Ok(true)
    ///     }
    /// }
    /// ```
    fn is_finished(
        &self,
        current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError>;
}
/// Planner for hovering at a fixed position
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::HoverPlanner;
/// let hover_planner = HoverPlanner {
///     target_position: Vector3::new(0.0, 0.0, 0.0),
///     target_yaw: 0.0,
/// };
/// ```
pub struct HoverPlanner {
    /// Target position for hovering
    pub target_position: Vector3<f32>,
    /// Target yaw angle for hovering
    pub target_yaw: f32,
}
/// Implementation of the `Planner` trait for the `HoverPlanner`
impl Planner for HoverPlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        _time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        (self.target_position, Vector3::zeros(), self.target_yaw)
    }

    fn is_finished(
        &self,
        _current_position: Vector3<f32>,
        _time: f32,
    ) -> Result<bool, SimulationError> {
        Ok(false) // Hover planner never "finished"
    }
}