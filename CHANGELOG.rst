^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package heron_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Bumped CMake version to avoid author warning.
* Minor linting changes.
* Switched to BSD licence.
* Removed message generation.
* Added GitHub CI, codeowners and issue template.
* Removed launch folder to be installed because it doesn't exist
* Switched to std_srvs/SetBool.
* Fixing dependencies
* Removed unnecessary includes
* Updated package.xml to format 2
* Deleted unnecessary log command
* Updated C++ version
* Updated Heron parameters
* Deleted unused files
* Updated PID params
* Last update to the default velocity covariance setting
* Making time-out the default option
* New control parameters for debugging
* Initialized output force to zero and changed default velocity covariance limit
* Added deadzone in motor control algorithms so motor's aren't always on
* Fixes to new command-switching system
* Added service to allow for disabling heron_controller
* Changed command timeout system
* Velocity timeout was checking the wrong covariance entry in the odom twist
* max_fwd_force already accounts for two thrusters
* Parameterized covariance check on sensor data
* Changed controll to only depend on odometry/filtered topic
* Added fwd vel PID control
* Contributors: Guy Stoppi, Tony Baltovski

0.1.0 (2016-07-05)
------------------
* Switched from updatePid to computeCommand as it is deprecated control_toolbox.
* Heron rename.
* Contributors: Tony Baltovski

0.0.5 (2014-03-05)
------------------
* add installation rule for header
* Contributors: Yan Ma

0.0.4 (2014-03-05)
------------------
* add include folder into catkin_package command
* Contributors: Yan Ma

0.0.3 (2013-10-24)
------------------
* Negate control outputs as a stopgap until kingfisher_controller correctly accounts for the base_link -> imu_link tf.

0.0.2 (2013-10-22)
------------------
* Launchfile fix for node name change.

0.0.1 (2013-10-17)
------------------
* Initial hydro release of kingfisher_controller, as extracted from kingfisher_node.
