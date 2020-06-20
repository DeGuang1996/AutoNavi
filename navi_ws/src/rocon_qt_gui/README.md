rocon_rqt_plugins
=================

Rqt graphical tools for rocon.

# Rocon Qt App Manager GUI

First, download the ```rocon_qt_qui```, and change the branch to ```implementation_app_manager_app```.

```
> cd <rocon workspace>/src
> git clone https://github.com/robotics-in-concert/rocon_qt_gui.git
> cd rocon_qt_gui
> git checkout implementation_app_manager_app
> yujin_make
> setup <rocon workspace>/devel/setup.bash
```

you can check the branch using ```git status```, and see the following message.
```
> cd <rocon workspace>/src/rocon_qt_gui
> git status
# On branch implementation_app_manager_app
```

Now you launch the chatter concert. 

```
> roslaunch rocon_app_manager standalone.launch --screen
```

And then start the app manager gui.
```
> rocon_qt_app_manager
```
