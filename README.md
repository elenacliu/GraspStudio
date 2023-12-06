This is a repository of my implentation of real robot grasp experiments on Franka Emika Robot. Now I only provide support on real Franka Emika robots, maybe simulators (like PyBullet) will be supported later for whom are potentially interested in robotics and EmbodiedAI but without accessible robots.

# Features

- 🌟 decoupled components
- 🌟 same code on all different robot configurations

# Code Structure

I decompose the whole grasping system into 4 components:

1. motion execution (of different control package), under directory `grasp`
2. end-effector execution (of different end-effectors, mostly grippers), under directory `end_effector`
3. motion solver, under `motion_solver`
4. camera/sensor input, under `cameras`

Each components is expected to be independent from others (Ideally! but sometimes it might be hard). If you want use your own modules, feel free to add your own class by inheriting the abstract class and implementing the necessary interfaces. 

Do not forget to implement the corresponding `Config` class for your class!

`grasp_franka_template.py` is one example of how to instantiate your own grasp class to execute grasp. For example, if you choose to execute grasp via `frankapy` pacakge's API, all you need to do is to create your own subclass `MyFrankaGrasp` and `MyFrankaGraspConfig`:


```python
class MyFrankaGrasp(FrankaGrasp):
  config: MyFrankaGraspConfig

  def __init__(self, config: MyFrankaGraspConfig):
    super().__init__(config=config)

  def grasp_once(self, method, **kwargs):
    """Implement your grasp algorithm here.
    """
    pass
```

In fact, you just need to implement the `grasp_once` function.

# Configuration System

I use the open-source repository [tyro](https://github.com/brentyi/tyro) as the configuration system. You can refer to its documentation for usage.

# Notes


## Grasp

### `franka-py` and `franka-interface`

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

# Acknowledgement

## Franka Packages

Frankapy: Zhang, Kevin, Mohit Sharma, Jacky Liang, and Oliver Kroemer. "A modular robotic arm control stack for research: Franka-interface and frankapy." arXiv preprint arXiv:2011.02398 (2020).

Pandapy: Elsner, J. (2023). Taming the Panda with Python: A powerful duo for seamless robotics programming and integration. SoftwareX, 24, 101532.

## Configuration Systems
Tyro: https://brentyi.github.io/tyro

## Simulators


# Miscs

The repository is under development. `requirements.txt` may not be complete. Feel free to contact `elena.cliu at gmail.com` if you meet any problem.



# TODOs

- [ ] add support for more motion solvers
- [ ] add support for pybullet
