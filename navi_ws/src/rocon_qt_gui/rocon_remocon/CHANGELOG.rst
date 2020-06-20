=========
Changelog
=========

Forthcoming
-----------
* [rocon_remocon] allow roslaunch style arg substitutions for global executables

0.9.1 (2016-10-12)
------------------
* [rocon_remocon] dont show the hidden group

0.8.1 (2016-03-16)
------------------
* [rocon_remocon] immediately show ui, even when no interactions manager yet found
* [rocon_remocon] bugfix saving of the group states for the ui combobox

0.8.0 (2015-10-10)
------------------
* major revamp to be similar to the nice qt app manager design
* support for starting/stopping one sided interactions
* support for interactions that depend on running rapps
* regression: no longer supports multimaster connections, needs to be FIXED

0.7.11 (2015-07-09)
-------------------
* [rocon_remocon] bypass role chooser if only one role.
* add firefox closes `#190 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/190>`_
* Contributors: Daniel Stonier, Jihoon Lee

0.7.10 (2015-04-27)
-------------------
* add missing dependency closes `#189 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/189>`_
* Contributors: Jihoon Lee

0.7.9 (2015-03-30)
------------------
* install scripts in local bin `#185 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/185>`_
* add resource instal rule closes `#185 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/185>`_
* Contributors: Jihoon Lee

0.7.8 (2015-03-23)
------------------
* fix message box argument into parent widget from None
* update QMessage box argument
* update way of interactions status update to remove qt warning message
* fix wrong parsor about ros host name and update shutdown process
* update setup.py to launch rqt_remocon in global
* delete unused resourece
* update api document
* update and test finish in rqt and rocon remocon
* make rqt remocon
* Contributors: dwlee

0.7.7 (2015-03-02)
------------------
* service proxy's are now get set properly.
* update connection way to improve slow connection issue
* Contributors: Jihoon Lee, dwlee

0.7.6 (2015-02-28)
------------------
* reverting the master checking process `#180 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/180>`_
* check all cached master status when it starts up. closes `#180 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/180>`_
* Contributors: Jihoon Lee

0.7.5 (2015-02-09)
------------------
* restore rocon_remocon src
* update ros service and subscriber init function
* make test brach
* Contributors: dwlee

0.7.4 (2014-12-30)
------------------

0.7.3 (2014-12-29)
------------------
* patch ui file search logic. `#172 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/172>`_
* Contributors: Jihoon Lee

0.7.2 (2014-11-21)
------------------

0.7.1 (2014-11-21)
------------------
* add web codename in remocon uri closes `#166 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/166>`_
* updates
* dialog creation refactored
* threadfied master check
* update logics in rocon_remocon
* Contributors: Jihoon Lee

0.7.0 (2014-08-25)
------------------
* don't raise exceptions when stopping dummy interactions for pairing, closes `#138 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/138>`_.
* Changed to get icon from rocon_icons instead of rocon_remocon
* Added icons for sub applications
* Added rocon_remocon Icon
* use rocon_launch to autogenerate a meta roslauncher, `#123 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/123>`_
* params to args for roslaunchables, closes `#127 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/127>`_.
* launching roslaunchers in their own terminals, `#123 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/123>`_.
* remappings for global executables.
* remove unused imports
* adding a stop all interactions button, closes `#120 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/120>`_
* skip role list if there is only one role, closes `#119 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/119>`_.
* handle case when no params for rosrunnable/global interactions.
* parameters for remocon rosrrunnables and globals, closes `#68 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/68>`_
* shutdown interactions when the interactions chooser shuts down.
* fix callback construction sequence (broke due to rocon/concert teleop
  refactoring).
* delay interactive client shutdown so remocon status can be receieved by the interactions manager.
* smooth interaction-role switching, `#26 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/26>`_.
* better feedback when the rocon master could not be found/communicated with, closes `#112 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/112>`_
* Merge branch 'indigo' into hydro-devel
* retreat to the master chooser if rocon interactions wasn't yet found, with a warning message box, fixes `#111 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/111>`_
* feedback for the case when a rosrunnable/global executable could not be found on the fileystem.
* feedback for the case when a roslaunch file could not be found on the filesystem.
* rocon_remocon: adds missing string variables in console output
* bugfix broken web browser lookup.
* minor bugfix to the remocon web browser callback and better feedback on teleop problems.
* don't abort if pairing support is not available.
* split main window code into different modules.
* update interactions list when stop interactions button is pressed.
* unused refresh button now marked as unused.
* signal gui updates across threads, fixes `#103 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/103>`_
* show the pairing interaction via background colour, `#98 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/98>`_
* pairing start/stop logic and exception handling (with message boxes) thrown in, `#98 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/98>`_
* get web broswer moved to utils and delected cpp style destructor (python doesn't need it).
* cleanup and handling for dummy paired interactions, but no automatic stop yet, `#98 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/98>`_
* launch info moved into a class and dummy launch type added with stub.
* checks collapsed into rocon_remocon and proper scripts discovery, fixes `#54 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/54>`_.
* provide the remocon name when requesting interactions, `#98 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/98>`_
* decouple rocon master lookups from the gui code.
* we can run more than one interaction in parallel, now we publish the fact, `#98 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/98>`_.
* web interaction handling, closes `#76 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/76>`_.
* update for interaction web app/url split.
* delete unused exception in concert checkup
* Added support for chromium browser
* get roles moved to a service
* fix the issue `#80 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/80>`_ and delete the unused log and chage the print metheod to using console module
* chagne the install role
* update the missing 78b19fab7913fcbc0d7eccf59b599b3678ae1f51 and change install file
* add the error exception handling when check up concert validation
* remove cache read and write api in remoconsub class
* remappings support for rosrunnables, closes `#69 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/69>`_
* concert -> ros master
* trivial cleanup.
* web apps specs finalised.
* properly convert python parameters and remappings into json url fields, `#63 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/63>`_
* minor simplification to the web app start
* support google-chrome-unstable
* temporary
* change the role of list stretch. I fix `#58 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/58>`_
* sort role list alphabetically, closes `#56 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/56>`_.
* shorter timeouts.
* remove unused remocon resources, `#55 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/55>`_
* minor comments, bugfix ghost subscriber variable appearance.
* get a filtered role list, `#52 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/52>`_
* rocon remocons using rocon icon packs...partially.
* trivial comment updates
* fix webapp loading (no master uri appending).
* hunt down interactions topics and services instead of defaulting to concert names.
* rocon remocon now independant of the concert.
* use unique hashes to populate internal lists, closes `#49 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/49>`_
* centralise home directory utils closes `#47 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/47>`_, increase checker timeout and simplify checker with subscriber proxy.
* basic working, of qt chatter, but logic errors still around.
* change show log about web app url
* change sniffing browser part, add sending parameter and remmaping info. to webapp
* change the exception part at determine the app type
* change the exception part
* add web launcher in remocon but only support chrome browser
* fix the exception error when finish the checkup process
* add a license
* kill process groups for global executables as well.
* support for rosrunnable and global executables, `#2 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/2>`_.
* adjustments to drop heir-part of uri if no concert name.
* some pep8 fixes, also make sure remocon window is on top, closes `#35 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/35>`_.
* multi-line concert name and concert connection info
* rocon_uri upgrades for rocon_remocon
* synchronised package versions.
* platform tuple overhaul.
* change add concert using master uri and host name. concert list update as soon as add concert
* change platform information at get app list part
* bugfix about the temp cache path
* disable the stop all apps button if there is no running app and change the button position in role list viewer
* add text box for settting the ros master uri and host name
* chagne the some button name and position in app list viewer
* change icon size bigger and text is smaller
* i fix `#17 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/17>`_
* i fixed Issue `#10 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/10>`_
* change start sub process method
* change the launch role that the already launched app is able to launch again
* change method of getting icon information and display the app icon
* code arrangement and delete app launcher scripts
* change the method of launching app and show the concert infomation in concert list viewer
* argument of host name bug fix and change the methon of subprocess terminate
* change save path to temporary path
* superflous launchers and remocon launch path bugfix.
* change unknown image format to png at check up scripts
* change unknown image format to png at check up scripts
* update the conduct graph as new message
* add validation checker about launch file
* add listener app for remocon and modify the app_launcher
* change image resources file, uuid to string uuid and code arrangment
* add parameter argument in start app launcher and code arrangement
* the timeout about waitting get role list set 1s
* add __init.py for launch without rosrun
* add time out at wait get role list part
* add argument abour host name when running the rocon remocon
* missing file update
* update
* remove broken install rule.
* upload setup.py and re-arrange the script files
* implementation of remocon sample frame
* Contributors: Daniel Stonier, DongWook Lee, Dongwook Lee, Gary Servin, Marcus Liebhardt, dwlee, kentsommer

0.5.4 (2013-09-11)
------------------

0.5.3 (2013-08-30)
------------------

0.5.2 (2013-07-17)
------------------

0.5.1 (2013-06-10 16:50:50 +0900)
---------------------------------

0.5.0 (2013-05-27)
------------------

0.3.1 (2013-04-09)
------------------

0.3.0 (2013-02-05)
------------------

0.2.0 (2013-01-31)
------------------
