#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from functools import partial
import json
import os
import types
import urllib
from urlparse import urlparse
import uuid
import yaml

from rocon_app_manager_msgs.msg import ErrorCodes
from rocon_console import console
import rocon_interactions
import rocon_interaction_msgs.msg as rocon_interaction_msgs
import rocon_interaction_msgs.srv as rocon_interaction_srvs
import rocon_interactions.web_interactions as web_interactions
import rocon_launch
import rocon_python_comms
import rocon_python_utils
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_uri
import rospkg
import rospy

from . import utils
from .launch import LaunchInfo, RosLaunchInfo
from .interactions import Interaction
from .interactions_table import InteractionsTable

##############################################################################
# Interactive Client Interface
##############################################################################


class InteractiveClientInterface(object):

    shutdown_timeout = 0.5

    """
    Time to wait before shutting down so remocon status updates can be received and processed
    by the interactions manager (important for pairing interactions).
    """

    def __init__(self, stop_interaction_postexec_fn):
        '''
          @param stop_app_postexec_fn : callback to fire when a listener detects an app getting stopped.
          @type method with no args
        '''
        self._interactions_table = InteractionsTable()
        self._stop_interaction_postexec_fn = stop_interaction_postexec_fn
        self.is_connect = False
        self.key = uuid.uuid4()
        self._ros_master_port = None
        try:
            self._roslaunch_terminal = rocon_launch.create_terminal()
        except (rocon_launch.UnsupportedTerminal, rocon_python_comms.NotFoundException) as e:
            console.warning("Cannot find a suitable terminal, falling back to the current terminal [%s]" % str(e))
            self._roslaunch_terminal = rocon_launch.create_terminal(rocon_launch.terminals.active)

        # this would be good as a persistant variable so the user can set something like 'Bob'
        self.name = "rqt_remocon_" + self.key.hex
        self.rocon_uri = rocon_uri.parse(
            rocon_uri.generate_platform_rocon_uri('pc', self.name) + "|" + utils.get_web_browser_codename()
        )
        # be also great to have a configurable icon...with a default
        self.platform_info = rocon_std_msgs.MasterInfo(version=rocon_std_msgs.Strings.ROCON_VERSION,
                                                       rocon_uri=str(self.rocon_uri),
                                                       icon=rocon_std_msgs.Icon(),
                                                       description=""
                                                       )
        self.currently_pairing_interaction_hashes = []
        self.active_one_sided_interaction = None

        # expose underlying functionality higher up
        self.interactions = self._interactions_table.generate_role_view
        """Get a dictionary of interactions belonging to the specified role."""

    def _connect_with_ros_init_node(self, ros_master_uri="http://localhost:11311", host_name='localhost'):
        # uri is obtained from the user, stored in ros_master_uri
        os.environ["ROS_MASTER_URI"] = ros_master_uri
        os.environ["ROS_HOSTNAME"] = host_name
        rospy.init_node(self.name, disable_signals=True)
        return self._connect(ros_master_uri, host_name)

    def _connect(self, ros_master_uri="http://localhost:11311", host_name='localhost'):

        self._ros_master_port = urlparse(os.environ["ROS_MASTER_URI"]).port
        console.logdebug("Interactive Client : Connection Details")
        console.logdebug("Interactive Client :   Node Name: " + self.name)
        console.logdebug("Interactive Client :   ROS_MASTER_URI: " + ros_master_uri)
        console.logdebug("Interactive Client :   ROS_HOSTNAME: " + host_name)
        console.logdebug("Interactive Client :   ROS_MASTER_PORT: %s" % self._ros_master_port)

        # Need to make sure we give init a unique node name and we need a unique uuid
        # for the remocon-role manager interaction anyway:

        try:
            console.logdebug("Interactive Client : Get interactions service Handle")
            interactions_namespace = rocon_python_comms.find_service_namespace('get_interactions', 'rocon_interaction_msgs/GetInteractions', unique=True)
            remocon_services = self._set_remocon_services(interactions_namespace)
        except rocon_python_comms.MultipleFoundException as e:
            message = "multiple interactions' publications and services [%s] are found. Please check services" % str(e)
            console.logerror("InteractiveClientInterface : %s" % message)
            return (False, message)
        except rocon_python_comms.NotFoundException as e:
            message = "failed to find all of the interactions' publications and services for [%s]" % str(e)
            console.logerror("InteractiveClientInterface : %s" % message)
            return (False, message)

        self.get_interactions_service_proxy = rospy.ServiceProxy(remocon_services['get_interactions'], rocon_interaction_srvs.GetInteractions)
        self.get_roles_service_proxy = rospy.ServiceProxy(remocon_services['get_roles'], rocon_interaction_srvs.GetRoles)
        self.request_interaction_service_proxy = rospy.ServiceProxy(remocon_services['request_interaction'], rocon_interaction_srvs.RequestInteraction)
        self.remocon_status_pub = rospy.Publisher("remocons/" + self.name, rocon_interaction_msgs.RemoconStatus, latch=True, queue_size=10)

        self.subscribers = rocon_python_comms.utils.Subscribers(
            [
                (interactions_namespace + "/pairing_status", rocon_interaction_msgs.PairingStatus, self._subscribe_pairing_status_callback)
            ]
        )

        self._publish_remocon_status()
        self.is_connect = True
        return (True, "success")

    def shutdown(self):
        if(not self.is_connect):
            return
        else:
            console.logdebug("Interactive Client : shutting down all interactions")
            for interaction in self._interactions_table.interactions:
                self.stop_interaction(interaction.hash)

            rospy.rostime.wallsleep(InteractiveClientInterface.shutdown_timeout)
            console.logdebug("Interactive Client : signaling shutdown.")
            rospy.signal_shutdown("shut down remocon_info")
            while not rospy.is_shutdown():
                rospy.rostime.wallsleep(0.1)

            console.logdebug("Interactive Client : has shutdown.")

    def get_role_list(self):
        if not self.is_connect:
            rospy.logwarn("InteractiveClientInterface : aborting a request to 'get_roles' as we are not connected to a rocon interactions manager.")
            return []
        try:
            response = self.get_roles_service_proxy(self.platform_info.rocon_uri)
        except (rospy.ROSInterruptException, rospy.ServiceException):
            return []
        return response.roles

    def get_runnable_interactions_list(self, role_name):
        """
        Contact the interactions manager and retrieve all the interactions
        associated to a particular role.

        :param str role_name: role to request list of interactions for.
        """
        filter_by_runtime_pairing_requirements = False
        call_result = self.get_interactions_service_proxy([role_name], self.platform_info.rocon_uri, filter_by_runtime_pairing_requirements)
        for msg in call_result.interactions:
            self._interactions_table.append(Interaction(msg))
        # could use a whole bunch of exception checking here, removing
        # self.interactions[role_name] if necessary.

    def has_running_interactions(self):
        """
        Identify if this client has any running interactions. Used by the gui above
        to enable/disable a button that will trigger stoppage of all running interactions.
        """
        for interaction in self._interactions_table.interactions:
            if interaction.launch_list.keys():
                return True
        return False

    def start_interaction(self, role_name, interaction_hash):
        """
        :param str interaction_hash: the key
        :param str role_name: help refine the search by specifying what role this interaction is for

        :returns: result of the effort to start an interaction, with a message if there was an error.
        :rtype: (bool, message)
        """
        interaction = self._interactions_table.find(interaction_hash)
        if interaction is None:
            return (False, "interaction key %s not found in interactions table" % interaction_hash)
        if interaction.role != role_name:
            return (False, "interaction key %s is in the interactions table, but under another role" % interaction_hash)

        # get the permission
        call_result = self.request_interaction_service_proxy(remocon=self.name, hash=interaction.hash)

        if call_result.error_code == ErrorCodes.SUCCESS:
            console.logdebug("Interactive Client : interaction request granted")
            try:
                (app_executable, start_app_handler) = self._determine_interaction_type(interaction)
            except rocon_interactions.InvalidInteraction as e:
                return False, ("invalid interaction specified [%s]" % str(e))
            result = start_app_handler(interaction, app_executable)
            if result:
                self._publish_remocon_status()
                if interaction.is_paired_type():
                    self.currently_pairing_interaction_hashes.append(interaction_hash)
                return (result, "success")
            else:
                return (result, "unknown")
        else:
            return False, ("interaction request rejected [%s]" % call_result.message)
        return (True, "success")

    def _determine_interaction_type(self, interaction):
        '''
          Classifies the interaction based on the name string and some intelligent
          (well, reasonably) parsing of that string.
           - paired dummy (by empty name)
           - ros launcher (by .launch extension)
           - ros runnable (by roslib find_resource success)
           - web app      (by web_interactions.parse)
           - web url      (by web_interactions.parse)
           - global executable (fallback option)
        '''
        # pairing trigger (i.e. dummy interaction)
        if not interaction.name:
            console.logdebug("Interactive Client : start a dummy interaction for triggering a pair")
            return ('', self._start_dummy_interaction)
        # roslaunch
        try:
            launcher_filename = rocon_python_utils.ros.find_resource_from_string(interaction.name, extension='launch')
            if interaction.remappings:
                raise rocon_interactions.InvalidInteraction("remappings are not yet enabled for roslaunchable interactions (workaround: try remapping via interaction parameters and roslaunch args)[%s]" % interaction.name)
            console.logdebug("Interactive Client : roslaunchable [%s]" % interaction.name)
            return (launcher_filename, self._start_roslaunch_interaction)
        except (rospkg.ResourceNotFound, ValueError):
            unused_filename, extension = os.path.splitext(interaction.name)
            if extension == '.launch':
                raise rocon_interactions.InvalidInteraction("could not find %s on the filesystem" % interaction.name)
            else:
                pass
        # rosrun
        try:
            rosrunnable_filename = rocon_python_utils.ros.find_resource_from_string(interaction.name)
            console.logdebug("Interactive Client : start_app_rosrunnable [%s]" % interaction.name)
            return (rosrunnable_filename, self._start_rosrunnable_interaction)
        except rospkg.ResourceNotFound:
            pass
        except Exception:
            pass
        # web url/app
        web_interaction = web_interactions.parse(interaction.name)
        if web_interaction is not None:
            if web_interaction.is_web_url():
                console.logdebug("Interactive Client : _start_weburl_interaction [%s]" % web_interaction.url)
                return (web_interaction.url, self._start_weburl_interaction)
            elif web_interaction.is_web_app():
                console.logdebug("Interactive Client : _start_webapp_interaction [%s]" % web_interaction.url)
                return (web_interaction.url, self._start_webapp_interaction)
        # executable
        if rocon_python_utils.system.which(interaction.name) is not None:
            console.logdebug("Interactive Client : _start_global_executable_interaction [%s]")
            return (interaction.name, self._start_global_executable_interaction)
        else:
            raise rocon_interactions.InvalidInteraction("could not find a valid rosrunnable or global executable for '%s' (mispelt, not installed?)" % interaction.name)

    def _start_dummy_interaction(self, interaction, unused_filename):
        console.loginfo("InteractiveClientInterface : starting paired dummy interaction")
        anonymous_name = interaction.name + "_" + uuid.uuid4().hex
        # process_listener = partial(self._process_listeners, anonymous_name, 1)
        # process = rocon_python_utils.system.Popen([rosrunnable_filename], postexec_fn=process_listener)
        interaction.launch_list[anonymous_name] = LaunchInfo(anonymous_name, True, None)  # empty shutdown function
        return True

    def _start_roslaunch_interaction(self, interaction, roslaunch_filename):
        '''
          Start a ros launchable application, applying parameters and remappings if specified.
        '''
        anonymous_name = interaction.display_name.lower().replace(" ", "_") + "_" + uuid.uuid4().hex
        launch_configuration = rocon_launch.RosLaunchConfiguration(
            name=roslaunch_filename,
            package=None,
            port=self._ros_master_port,
            title=interaction.display_name,
            namespace=interaction.namespace,
            args=self._prepare_roslaunch_args(interaction.parameters),
            options="--screen"
        )
        process_listener = partial(self._process_listeners, anonymous_name, 1)
        (process, meta_roslauncher) = self._roslaunch_terminal.spawn_roslaunch_window(launch_configuration, postexec_fn=process_listener)
        interaction.launch_list[anonymous_name] = RosLaunchInfo(anonymous_name, True, process, self._roslaunch_terminal.shutdown_roslaunch_windows, [meta_roslauncher])
        return True

    def _start_rosrunnable_interaction(self, interaction, rosrunnable_filename):
        '''
          Launch a rosrunnable application. This does not apply any parameters
          or remappings (yet).
        '''
        # the following is guaranteed since we came back from find_resource calls earlier
        # note we're overriding the rosrunnable filename here - rosrun doesn't actually take the full path.
        package_name, rosrunnable_filename = interaction.name.split('/')
        name = os.path.basename(rosrunnable_filename).replace('.', '_')
        anonymous_name = name + "_" + uuid.uuid4().hex
        process_listener = partial(self._process_listeners, anonymous_name, 1)
        cmd = ['rosrun', package_name, rosrunnable_filename, '__name:=%s' % anonymous_name]
        remapping_args = []
        for remap in interaction.remappings:
            remapping_args.append(remap.remap_from + ":=" + remap.remap_to)
        cmd.extend(remapping_args)
        cmd.extend(self._prepare_command_line_parameters(interaction.parameters))
        console.logdebug("Interactive Client : rosrunnable command %s" % cmd)
        process = rocon_python_utils.system.Popen(cmd, postexec_fn=process_listener)
        interaction.launch_list[anonymous_name] = LaunchInfo(anonymous_name, True, process)
        return True

    def _start_global_executable_interaction(self, interaction, filename):
        console.logwarn("Interactive Client : starting global executable [%s]" % interaction.name)
        name = os.path.basename(filename).replace('.', '_')
        anonymous_name = name + "_" + uuid.uuid4().hex
        process_listener = partial(self._process_listeners, anonymous_name, 1)
        cmd = [filename]
        remapping_args = []
        for remap in interaction.remappings:
            remapping_args.append(remap.remap_from + ":=" + remap.remap_to)
        cmd.extend(remapping_args)
        cmd.extend(self._prepare_command_line_parameters(interaction.parameters))
        console.logdebug("Interactive Client : global executable command %s" % cmd)
        process = rocon_python_utils.system.Popen(cmd, postexec_fn=process_listener)
        interaction.launch_list[anonymous_name] = LaunchInfo(anonymous_name, True, process)
        return True

    def _start_weburl_interaction(self, interaction, url):
        """
        We only need the url here and then do a system check for a web browser.
        """
        web_browser = utils.get_web_browser()
        if web_browser is not None:
            name = os.path.basename(web_browser).replace('.', '_')
            anonymous_name = name + "_" + uuid.uuid4().hex
            process_listener = partial(self._process_listeners, anonymous_name, 1)
            process = rocon_python_utils.system.Popen([web_browser, "--new-window", url], postexec_fn=process_listener)
            interaction.launch_list[anonymous_name] = LaunchInfo(anonymous_name, True, process)
            return True
        else:
            return False

    def _start_webapp_interaction(self, interaction, base_url):
        """
        Need to work out the extended url (with args, parameters and remappings) here and then feed that to a
        detected browser.

        :param base_url str: the web app url without all of the attached variables.
        """
        web_browser = utils.get_web_browser()
        if web_browser is not None:
            url = self._prepare_webapp_url(interaction, base_url)
            name = os.path.basename(web_browser).replace('.', '_')
            anonymous_name = name + "_" + uuid.uuid4().hex
            process_listener = partial(self._process_listeners, anonymous_name, 1)
            process = rocon_python_utils.system.Popen([web_browser, "--new-window", url], postexec_fn=process_listener)
            interaction.launch_list[anonymous_name] = LaunchInfo(anonymous_name, True, process)
            return True
        else:
            return False

    def stop_all_interactions(self):
        """
        This is the big showstopper - stop them all!
        """
        console.logdebug("Interactive client : stopping all interactions")
        running_interactions = []
        for interaction in self._interactions_table.interactions:
            for unused_process_name in interaction.launch_list.keys():
                running_interactions.append(interaction.hash)
        for interaction_hash in running_interactions:
            self.stop_interaction(interaction_hash)

    def stop_interaction(self, interaction_hash):
        """
        This stops all launches for an interaction of a particular type.
        """
        interaction = self._interactions_table.find(interaction_hash)
        if interaction is None:
            error_message = "interaction key %s not found in interactions table" % interaction_hash
            console.logwarn("Interactive Client : could not stop interactions of this kind [%s]" % error_message)
            return (False, "%s" % error_message)
        else:
            console.logdebug("Interactive Client : stopping all '%s' interactions" % interaction.display_name)
        try:
            for launch_info in interaction.launch_list.values():
                if launch_info.running:
                    launch_info.shutdown()
                    console.loginfo("Interactive Client : interaction stopped [%s]" % (launch_info.name))
                    del interaction.launch_list[launch_info.name]
                elif launch_info.process is None:
                    launch_info.running = False
                    console.loginfo("Interactive Client : no attached interaction process to stop [%s]" % (launch_info.name))
                    del interaction.launch_list.launch_list[launch_info.name]
                else:
                    console.loginfo("Interactive Client : interaction is already stopped [%s]" % (launch_info.name))
                    del interaction.launch_list.launch_list[launch_info.name]
        except Exception as e:
            console.logerror("Interactive Client : error trying to stop an interaction [%s][%s]" % (type(e), str(e)))
            # this is bad...should not create bottomless exception buckets.
            return (False, "unknown failure - (%s)(%s)" % (type(e), str(e)))
        # console.logdebug("Interactive Client : interaction's updated launch list- %s" % str(interaction.launch_list))
        if interaction.is_paired_type():
            self.currently_pairing_interaction_hashes = [h for h in self.currently_pairing_interaction_hashes if h != interaction_hash]
        self._publish_remocon_status()
        return (True, "success")

    def _process_listeners(self, name, exit_code):
        '''
          This is usually run as a post-executing function to the interaction and so will do the
          cleanup when the user's side of the interaction has terminated.

          Note that there are other means of stopping & cleanup for interactions:

          - via the remocon stop buttons (self.stop_interaction, self.stop_all_interactions)
          - via a rapp manager status callback when it is a pairing interaction

          There is some common code (namely del launch_list element, check pairing, publish remocon) so
          if changing that flow, be sure to check the code in self.stop_interaction()

          @param str name : name of the launched process stored in the interactions launch_list dict.
          @param int exit_code : could be utilised from roslaunched processes but not currently used.
        '''
        terminated = False
        for interaction in self._interactions_table.interactions:
            if name in interaction.launch_list:
                del interaction.launch_list[name]
                # toggle the pairing indicator if it was a pairing interaction
                if interaction.is_paired_type() and interaction.hash in self.currently_pairing_interaction_hashes:
                    self.currently_pairing_interaction_hashes = [interaction_hash for interaction_hash in self.currently_pairing_interaction_hashes if interaction_hash != interaction.hash]
                if not interaction.launch_list:
                    # inform the gui to update
                    self._stop_interaction_postexec_fn()
                # update the rocon interactions handler
                self._publish_remocon_status()
                terminated = True
                break
        if not terminated:
            console.logwarn("Interactive Client : detected a terminating interaction, but nothing to do [either mopped up via the gui or via rapp manager callback][%s]" % name)
        else:
            console.logdebug("Interactive Client : detected terminating interaction and moppedddd up appropriately [%s]" % name)

    ######################################
    # Ros Comms
    ######################################

    def _publish_remocon_status(self):
        remocon_status = rocon_interaction_msgs.RemoconStatus()
        remocon_status.platform_info = self.platform_info
        remocon_status.uuid = str(self.key.hex)
        remocon_status.version = rocon_std_msgs.Strings.ROCON_VERSION
        running_interactions = []
        for interaction in self._interactions_table.interactions:
            for unused_process_name in interaction.launch_list.keys():
                running_interactions.append(interaction.hash)
        remocon_status.running_interactions = running_interactions
        console.logdebug("Interactive Client : publishing remocon status")
        self.remocon_status_pub.publish(remocon_status)

    def _subscribe_pairing_status_callback(self, msg):
        console.logdebug("Interactive Client : pairing events callback")
        rapp_stopped = not msg.is_managing_paired_interactions
        if rapp_stopped:
            for interaction_hash in self.currently_pairing_interaction_hashes:
                console.logdebug("Interactive Client : the rapp in this paired interaction terminated [%s]" % interaction_hash)
                self.stop_interaction(interaction_hash)
            # update the gui -> DJS: update the gui if it started OR stopped with the new filtered interactions list
            self.currently_pairing_interaction_hashes = []
            self._stop_interaction_postexec_fn()
        if msg.is_managing_one_sided_interaction:
            self.active_one_sided_interaction = msg.active_one_sided_interaction
            rospy.logdebug("Interactive Client : is managing a core one sided interaction [%s]" % msg.active_one_sided_interaction)
        else:
            self.active_one_sided_interaction = None

    ######################################
    # Utilities
    ######################################

    def _set_remocon_services(self, interactions_namespace):
        """
            setting up remocon-interaction manager apis. and check if the services are available

            :param str interactions_namespace : namespace to contact interaction manager

            :returns: remocon service apis
            :rtype: dict
        """
        remocon_services = {}
        remocon_services['get_interactions'] = interactions_namespace + '/' + 'get_interactions'
        remocon_services['get_roles'] = interactions_namespace + '/' + 'get_roles'
        remocon_services['request_interaction'] = interactions_namespace + '/' + 'request_interaction'

        for service_name in remocon_services.keys():
            if not rocon_python_comms.service_is_available(remocon_services[service_name]):
                raise rocon_python_comms.NotFoundException("'%s' service is not validated" % service_name)

        return remocon_services

    def _prepare_webapp_url(self, interaction, base_url):
        """
           url synthesiser for sending remappings and parameters information.
           We convert the interaction parameter (yaml string) and remapping (rocon_std_msgs.Remapping[])
           variables into generic python list/dictionary objects and convert these into
           json strings as it makes it easier for web apps to handle them.
        """
        interaction_data = {}
        interaction_data['display_name'] = interaction.display_name
        # parameters
        interaction_data['parameters'] = yaml.load(interaction.parameters)
        # remappings
        interaction_data['remappings'] = {}  # need to create a dictionary for easy parsing (note: interaction.remappings is a list of rocon_std_msgs.Remapping)
        for r in interaction.remappings:
            interaction_data['remappings'][r.remap_from] = r.remap_to
        # package all the data in json format and dump it to one query string variable
        console.logdebug("Remocon Info : web app query string %s" % interaction_data)
        query_string_mappings = {}
        query_string_mappings['interaction_data'] = json.dumps(interaction_data)
        # constructing the url
        return base_url + "?" + urllib.urlencode(query_string_mappings)

    def _prepare_command_line_parameters(self, interaction_parameters):
        """
        Convert the interaction specified yaml string into command line parameters that can
        be passed to rosrunnable or global nodes.

        :param str interaction_parameters: parameters specified as a yaml string

        :returns: the parameters as command line args
        :rtype: str[]
        """
        parameters = []
        parameter_dictionary = yaml.load(interaction_parameters)  # convert from yaml string into python dictionary
        if parameter_dictionary is not None:  # None when there is no yaml configuration
            for name, value in parameter_dictionary.items():
                if type(value) is types.DictType or type(value) is types.ListType:
                    parameters.append('_' + name + ':=' + yaml.dump(value))
                else:  # it's a dict or list, so dump it
                    parameters.append('_' + name + ':=' + str(value))
        return parameters

    def _prepare_roslaunch_args(self, interaction_parameters):
        """
        Convert the interaction specified yaml string into roslaunch args
        to be passed to the roslaunchable. Note that we only use a constrained
        subset of yaml to be compatible with roslaunch args here.
        The root type has to be a dict and values themselves
        may not be dicts or lists.

        :param str interaction_parameters: parameters specified as a yaml string

        :returns: the parameters as roslaunch args key-value pairs
        :rtype: list of (name, value) pairs
        """
        args = []
        parameters = yaml.load(interaction_parameters)  # convert from yaml string into python dictionary
        if parameters is not None:
            if type(parameters) is types.DictType:
                for name, value in parameters.items():
                    if type(value) is types.DictType or type(value) is types.ListType:
                        console.logwarn("Ignoring invalid parameter for roslaunch arg (simple key-value pairs only) [%s][%s]" % (name, value))
                    else:
                        args.append((name, value))
            else:
                console.logwarn("Ignoring invalid parameters for roslaunch args (must be a simple key-value dict) [%s]" % parameters)
        return args
