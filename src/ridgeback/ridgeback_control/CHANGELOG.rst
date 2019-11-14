^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ridgeback_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2019-03-25)
------------------
* Changed boost::shared_ptr to urdfdom shared pointers
* ridgeback_control: fixed missing calculation of wheels_k\_ (regression from daa4bc9050f786ed092fab911dd217da6febeae0)
* Simplify boolean logic for wheel separation checks
* Allow URDF to be optional
* Contributors: Catherine Wong, Chad Rockey, Johannes Meyer, Tony Baltovski

0.2.1 (2018-04-12)
------------------
* Added support for planar motion using interactive markers.
* [ridgeback_control] Made PS4 default controller.
* Contributors: Tony Baltovski

0.2.0 (2018-04-12)
------------------
* Removed tight default rolling window
* Updated rolling window for odom responsiveness.  Minor changes to control and urdf syntax for kinetic
* Updated to Package format 2.
* [ridgeback_control] Added ability to override default control parameters with environment variables.
* Contributors: Dave Niewinski, Tony Baltovski

0.1.10 (2017-06-26)
-------------------

0.1.9 (2017-04-17)
------------------
* Updated maintainer.
* Contributors: Tony Baltovski

0.1.8 (2016-09-30)
------------------

0.1.7 (2016-07-18)
------------------
* Fixed teleop angular axis for PS4.
* Contributors: Tony Baltovski

0.1.6 (2016-05-25)
------------------
* Added support for PS4 controller.
* Contributors: Tony Baltovski

0.1.5 (2016-04-22)
------------------

0.1.4 (2016-04-18)
------------------

0.1.3 (2016-03-02)
------------------
* Updated URDF for physical changes.
* Updated control limits.
* Removed yaw estimate from robot_localization.
* Contributors: Tony Baltovski

0.1.2 (2015-12-22)
------------------
* Added wheel separation override.
* Contributors: Tony Baltovski

0.1.1 (2015-12-01)
------------------

0.1.0 (2015-11-19)
------------------
* Initial ridgeback release.
* Contributors: Mike Purvis, Tony Baltovski
