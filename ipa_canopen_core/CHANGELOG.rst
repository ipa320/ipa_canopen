^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ipa_canopen_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.4 (2014-04-08)
------------------
* Fixes the move_device and homing tools for the multiple chains version
* Small clean
* Seeing EMCY
* Seeing EMCY
* Some init chains
* First functional and hardware tested version of the multiple_chains node, fixes `ipa320/ipa_canopen#57 <https://github.com/ipa320/ipa_canopen/issues/57>`_
* Fixes `ipa320/ipa_canopen#48 <https://github.com/ipa320/ipa_canopen/issues/48>`_
* Show devices on bus for get_error
* Incredible boost to initialization speed
* Contributors: thiagodefreitas

0.5.3 (2014-03-28)
------------------
* Lint Fixes for ipa_canopen_core
* Catkin_lint modifications
  *Init bug fix
  *Schunk bug fix
* Contributors: thiagodefreitas

0.5.2 (2014-03-28)
------------------
* merged
* revert pcan as external dependency
* merged
* Fixes schunk node
* Merge pull request `#33 <https://github.com/ipa320/ipa_canopen/issues/33>`_ from thiagodefreitas/hydro_dev
  Last day for syncs between groovy_dev and hydro_dev
* Fixes some installation problems acording to catkin_lint
* catkin_make_isolated is now working
* merge and smarter init
* Last day for syncs between groovy_dev and hydro_dev
* Syncs the hydro_dev to the groovy_dev
* transmit mode of operation SDO as uint8_t
* use int8_t for mode of operation
* allow motor state to be changed in both directions
* change mode of operation from uint8_t to int8_t
* function to set operation mode
* make pcan an external dependency
* overloaded init function to allow initialization with multiple modes
* setMotorState automatically goes through necessary states
* changed timing for change of operation mode
* Fixing author name
* Fixed delays when initializing
* Faster boot-up for Elmo
* small fixes for working with Schunk
* Restricted some calls to the core part
* Fixing the TPCanRdmsg
* Fixing syscall param print to uninitialised bytes
* Some beautifying
* Removed some unnecessary prints
* Missing files
* Fixing merge errors
* Tested on cob4 torso
* Fixed move device with mapped Schunk device
* Almost merged schunk and elmo
* Testing gtest for the ipa_canopen_core
* Update from the manual
* Modifying structure for 3 joints
* Position mode working at industrieStr
* Added swithces
* Changing Profiled Position Mode to a ROS Service
* Probably solves the errors from industrieStr
* Modifying the offsets and conversion factors
* Unit factor now comes from the yaml file
* Some cleaning
* Working for different ranges of baud rates
* Mapping is now independent of the canopen id
* Merged from changes at industrieStr
* Local changes
* elmo_pos worked for the first time
* Removing hard-coded baudrate from low-level Canopen
* Trying things
* fixes on cob3-7
* Std::couts out
* Removing some comments
* Definitions for the sendVel
* Functions separation between sendPos e sendVel
* Separating sendPos to sendVel
* Changes at sendPos
* No more fixed IDs for the Elmo Branch
* Correcting elmo endschalten
* Limits working properly, only the switch release needs some adjustment
* Hardware Limit Switches status
* Adjusting comments and license for the Elmo parts of the driver
* Recover works for the first time
* Elmo merging
* Contributors: Kai Franke, Thiago de Freitas, Thiago de Freitas Oliveira Araujo, cob4-1, ipa-cob3-7, thiagodefreitas

0.5.1 (2014-03-20)
------------------
* This commit syncs the groovy_dev branch with the hydro_dev branch
* fix CMakeLists.txt
* Modifying Cmake
* Problem with library name
* Small mistake on the package.xml
* Changed CMakeLists to just one
* Modifications from cob3
* Catkinized Version of the ipa_canopen package
* Starting the catkinize process for the canopen stack
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
* Schunk errors description
* Some printout cleaning
* Velocity limit check for ROS
* Modified ROS part
* Pushing for saving
* Still only static recover
* FAULT_REACTION_ACTIVE
* Recover for static
* Deleted differente CMakeLists
* Florian modifications from Jenkins warnings
* Modifications tested with the LWA 4.10
* remove compiler warning
* Driver modifications
* Merge branch 'electric_dev' of github.com:uhr-eh/ipa_canopen into origin-thiago/electric_dev
  Conflicts:
  ipa_canopen_core/driver/canopen.cpp
* Implementing the diagnostics
* updated 64Bit version of ipa_canopen
* updated 64Bit version of ipa_canopen
* updated 64Bit version of canopen driver
* first version for 64Bit OS
* updated 64Bit version
* first test-version for ubuntu12 64bit
* first test-version for ubuntu12 64bit
* updated state machines
* updated state machines
* updated state machines
* updated state machines
* updated motor state machine
* updated NMT state machine and nodeguard handling
* updated NMT state machine and nodeguard handling
* updated NMT State machine and nodeguard handling
* updated NMT state machine & nodeguard handling
* updated NMT machine & nodeguarding function
* updated NWT state machine
* updated NWT state machine
* updated NWT state machine
* updated NWT state machine
* updated NWT state machine
* updated NMT state machine
* updated state NMW state machine
* added some docs on Schunk powerball arm
* added some documentation for Schunk Powerball arm
* updated stack and package info
* updated documentation
* small fixes
* updated documentation
* updated documentation
* moved documentation folder
* building for ros and non ros
* first step for building with rosmake
* added canopen core to repository
* Contributors: Thiago de Freitas, ipa-cob3-3, ipa-fmw, ipa-fxm, ipa-tys, ipa-uhr-eh, thiago, uhr-eh
