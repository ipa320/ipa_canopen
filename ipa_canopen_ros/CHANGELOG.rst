^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ipa_canopen_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.6 (2014-05-28)
------------------
* Error install tags
* Contributors: ipa-cob4-1

0.5.5 (2014-05-21)
------------------
* change warning message
* merge conflict
* hydro
* Diagnostics with name
* Diagnostics with name
* Diagnostics with name
* Adjusts the diagnostics
* Merge pull request `#58 <https://github.com/ipa320/ipa_canopen/issues/58>`_ from thiagodefreitas/hydro_dev
  Changes the behaviour when a node can not be found
* Changes the behaviour when a node can not be found
* Adds some diagnostics
* Fixes init bug
* Fixes problems for the inialization of Schunk devices
* First version of generalized mapping functions
* Contributors: Thiago de Freitas Oliveira Araujo, ipa-cob4-1, ipa-nhg, ros, thiagodefreitas

0.5.4 (2014-04-08)
------------------
* Small clean
* Some init chains
* Changes on diagnostics
* fixes `ipa320/cob4#58 <https://github.com/ipa320/cob4/issues/58>`_
* First functional and hardware tested version of the multiple_chains node, fixes `ipa320/ipa_canopen#57 <https://github.com/ipa320/ipa_canopen/issues/57>`_
* still fixes `ipa320/cob4#58 <https://github.com/ipa320/cob4/issues/58>`_
* fixes `ipa320/cob4#58 <https://github.com/ipa320/cob4/issues/58>`_
* fixes `ipa320/cob4#58 <https://github.com/ipa320/cob4/issues/58>`_
* fixes `ipa320/ipa_canopen#45 <https://github.com/ipa320/ipa_canopen/issues/45>`_
* Contributors: thiagodefreitas

0.5.3 (2014-03-28)
------------------
* Merge branch 'hydro_dev' of github.com:ipa320/ipa_canopen into hydro_release_candidate
* Catkin_lint modifications
  *Init bug fix
  *Schunk bug fix
* Contributors: thiagodefreitas

0.5.2 (2014-03-28)
------------------
* Urgent Fix to ros pre-release build
* Missing a comment
* Changes the checks from the CMakeLists equal to ipa_canopen_ros
* Merge pull request `#36 <https://github.com/ipa320/ipa_canopen/issues/36>`_ from thiagodefreitas/hydro_dev
  Added diagnostic_msgs
* Added diagnostic_msgs
* Changes executable name to be compliant with catkin specs
* catkin_make_isolated is now working
* Last day for syncs between groovy_dev and hydro_dev
* Syncs the hydro_dev to the groovy_dev
* Fixing author name
* Fixes bug described by @kalectro
* Changed pr2 msgs to control_msgs
* Restricted some calls to the core part
* Fixing the TPCanRdmsg
* Removed some elmo trash
* Some beautifying
* Removed some unnecessary prints
* Ros Node Working for all the 3 devices
* Fixing the ros node
* Fixing merge errors
* Merge branch 'feature/simple_bride' of github.com:thiagodefreitas/ipa_canopen into almost_merged
  Conflicts:
  ipa_canopen_ros/CMakeLists.txt
  ipa_canopen_ros/src/canopen_ros.cpp
* integrate init, recover and moveVel. still not working yet
* add checks for ROS parameters
* add diagnostic_msgs to manifest and stack
* demo for torso
* Added PPMode service
* Modifying structure for 3 joints
* Position mode working at industrieStr
* Added swithces
* Removed sendPos
* devices
* Default positions for the request
* Changing Profiled Position Mode to a ROS Service
* Probably solves the errors from industrieStr
* Modifying the offsets and conversion factors
* Unit factor now comes from the yaml file
* Working for different ranges of baud rates
* Mapping is now independent of the canopen id
* Merged from changes at industrieStr
* Local changes
* elmo_pos worked for the first time
* Removing hard-coded baudrate from low-level Canopen
* Std::couts out
* Definitions for the sendVel
* Separating sendPos to sendVel
* Changed Handlers at ROS Level
* No more fixed IDs for the Elmo Branch
* Correcting elmo endschalten
* Limits working properly, only the switch release needs some adjustment
* Adjusting comments and license for the Elmo parts of the driver
* Recover works for the first time
* Changed elmo constraints
* Elmo merging
* Contributors: Thiago de Freitas, Thiago de Freitas Oliveira Araujo, ipa-fmw, thiagodefreitas

0.5.1 (2014-03-20)
------------------
* This commit syncs the groovy_dev branch with the hydro_dev branch
* Changed CMakeLists to just one
* Modifications from cob3
* Catkinized Version of the ipa_canopen package
* Merge pull request `#21 <https://github.com/ipa320/ipa_canopen/issues/21>`_ from thiagodefreitas/groovy_dev
  ROS node uses offsets for desired and actual positions
* Correction for the desired positions
* Modifying offsets
* fix recover and init behaviour
* Updating author and maintainer information
* Renamed function and services from stop to Halt
* Example of stop service
* Recover on movement now works
* Enhanced diagnostics version
* Pre initialization information
  New functions for getting the manufacturer erros
* Manufacturer information:
  * hardware version
  *firmware version
* Some printout cleaning
* Changing headers
* Velocity limit check for ROS
* Modified ROS part
* Pushing for saving
* Still only static recover
* remove yaml-cpp
* More jenkins changes
* Florian modifications from Jenkins warnings
* Modifications tested with the LWA 4.10
* add yaml-cpp dep
* Modified
* Deleted trash
* Driver modifications
* Implementing the diagnostics
* updated 64Bit version of ipa_canopen
* updated 64Bit version of ipa_canopen
* updated 64Bit version of canopen driver
* first version for 64Bit OS
* updated 64Bit version
* first test-version for ubuntu12 64bit
* updated stack and package info
* small fixes
* first step for building with rosmake
* added canopen_ros
* Contributors: Florian Weißhardt, Thiago de Freitas, ipa-cob3-3, ipa-cob3-5, ipa-fmw, ipa-tys, ipa-uhr-eh, thiago, uhr-eh
