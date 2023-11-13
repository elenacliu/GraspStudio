This is a repository of my implentation of some real robot grasp experiments on Franka Emika Robot.

# Some useful functions of `panda-py`

1. get position of end-effector

```
get_position(self: panda_py._core.Panda) → numpy.ndarray[numpy.float64[3, 1]]

    Current end-effector position in robot base frame.

```

2. get joint of robot arm

```
@property
def q(self) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]:
    """
    :type: numpy.ndarray[numpy.float64, _Shape[7, 1]]
    """
pass
```


# Notes of `franka-py` and `franka-interface`

You should first enter the directory of frankapy and start the control pc. (In my case, the control pc is the same with the client pc)

```bash
cd /path/to/frankapy
conda activate your_env_that_contains_frankapy
bash ./bash_scripts/start_control_pc.sh -i localhost
```

Sometimes you might want to change the mode to "user mode". The franka-interface in the control pc may get down in this mode, so you should close the terminals and rerun `bash ./bash_scripts/start_control_pc.sh -i localhost`.

Sometimes the arm will suddenly stop (but joints do not exceed the upper and lower limits), that's possibly because it collides with "virtual wall" defined in code repository:

```c++
// franka-interface code
void TerminationHandler::check_terminate_virtual_wall_collisions(const franka::RobotState &robot_state, franka::Model *robot_model) {
  if (!done_) {
    int n_frame = 0;
    // Check all joints that are not base or EE
    for (franka::Frame frame = franka::Frame::kJoint2; frame <= franka::Frame::kFlange; frame++) {
      std::array<double, 16> pose = robot_model->pose(frame, robot_state);
      Eigen::Vector3d pos(pose[12], pose[13], pose[14]);
    
      for (uint n_plane = 0; n_plane < planes_.size(); n_plane++) {
        double dist = planes_[n_plane].absDistance(pos);
        
        if (dist < dist_thresholds_[n_frame]) {
          std::cout << "Frame " << n_frame + 1 << "is in collision with wall" << n_plane << "with distance " << dist << std::endl;
          done_ = true;
          terminated_by_virt_coll_ = true;
          break;
        }
      }

      n_frame++;
      if (done_) { break; }
    }
  }
}
```

so you should modify the virtual wall value to your need:

```c++
  // Create hyperplanes
  const std::vector<Eigen::Hyperplane<double,3>> planes_ {
    //// original virtual walls
    // // Front
    // Eigen::Hyperplane<double, 3>(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0.75, 0., 0.)),
    // // Sides 
    // Eigen::Hyperplane<double, 3>(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0.47, 0.)),
    // Eigen::Hyperplane<double, 3>(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., -0.47, 0.)),
    // // Bottom
    // Eigen::Hyperplane<double, 3>(Eigen::Vector3d(0., 0., 1.), Eigen::Vector3d(0., 0., -0.015)),
    // // Top
    // Eigen::Hyperplane<double, 3>(Eigen::Vector3d(0., 0., 1.), Eigen::Vector3d(0., 0., 1.25)),
    // // Back
    // Eigen::Hyperplane<double, 3>(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(-0.46, 0., 0.))

    //// virtual walls of hyperplane lab
    // Front
    Eigen::Hyperplane<double, 3>(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(1.2, 0., 0.)),
    // Sides 
    Eigen::Hyperplane<double, 3>(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0.8, 0.)),
    Eigen::Hyperplane<double, 3>(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., -0.8, 0.)),
    // Bottom
    Eigen::Hyperplane<double, 3>(Eigen::Vector3d(0., 0., 1.), Eigen::Vector3d(0., 0., -0.015)),
    // Top
    Eigen::Hyperplane<double, 3>(Eigen::Vector3d(0., 0., 1.), Eigen::Vector3d(0., 0., 1.25)),
    // Back
    Eigen::Hyperplane<double, 3>(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(-0.46, 0., 0.))
  };

```

then just re-build the `franka-interface` package and enjoy your code!

# Miscs

The repository is under development. `requirements.txt` may not be complete. Feel free to contact `elena.cliu at gmail.com` if you meet any problem.

# TODOs

1. add support for pybullet
2. pip requirements.txt sync
3. readme
  - how to use the repo
  - how to install the required environment