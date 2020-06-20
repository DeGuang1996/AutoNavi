#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import re
import copy
import rocon_gateway_utils
import rosgraph.impl.graph
import roslib

##############################################################################
# Implementation
##############################################################################

# Show gateway connections where gateway flips/pulls are active
GATEWAY_GATEWAY_GRAPH = 'gateway_gateway'
# Show pulled connections between the gateways
GATEWAY_PULLED_GRAPH = 'gateway_pulled'
# Show flipped connections between the gateways
GATEWAY_FLIPPED_GRAPH = 'gateway_flipped'


def matches_any(name, patternlist):
    if patternlist is None or len(patternlist) == 0:
        return False
    for pattern in patternlist:
        if str(name).strip() == pattern:
            return True
        if re.match("^[a-zA-Z0-9_/]+$", pattern) is None:
            if re.match(str(pattern), name.strip()) is not None:
                return True
    return False


class NodeConnections:
    def __init__(self, incoming=None, outgoing=None):
        self.incoming = incoming or []
        self.outgoing = outgoing or []


class RosGraphDotcodeGenerator:

    def __init__(self):
        pass

    def _add_edge(self, edge, dotcode_factory, dotgraph, is_topic=False):
        if is_topic:
            dotcode_factory.add_edge_to_graph(dotgraph, edge.start, edge.end, label=edge.label, url='topic:%s' % edge.label)
        else:
            dotcode_factory.add_edge_to_graph(dotgraph, edge.start, edge.end, label=edge.label)

    def _add_node(self, node, rosgraphinst, dotcode_factory, dotgraph):
        if node in rosgraphinst.bad_nodes:
            bn = rosgraphinst.bad_nodes[node]
            if bn.type == rosgraph.impl.graph.BadNode.DEAD:
                dotcode_factory.add_node_to_graph(dotgraph,
                                                  nodename=node,
                                                  shape="doublecircle",
                                                  url=node,
                                                  color="red")
            else:
                dotcode_factory.add_node_to_graph(dotgraph,
                                                  nodename=node,
                                                  shape="doublecircle",
                                                  url=node,
                                                  color="orange")
        else:
            dotcode_factory.add_node_to_graph(dotgraph,
                                              nodename=node,
                                              #nodename=rocon_utilities.gateway_basename(node),
                                              #nodelabel=rocon_utilities.gateway_basename(node),
                                              shape='ellipse',
                                              url=rocon_gateway_utils.gateway_basename(node),
                                              #url=node
                                              )

    def _add_topic_node(self, node, dotcode_factory, dotgraph):
        label = rosgraph.impl.graph.node_topic(node)
        dotcode_factory.add_node_to_graph(dotgraph,
                                          nodename=label,
                                          nodelabel=label,
                                          shape='box',
                                          url="topic:%s" % label)

    def generate_namespaces(self, graph, graph_mode):
        """
        Determine the namespaces of the nodes being displayed
        """
        namespaces = []
        if graph_mode == GATEWAY_GATEWAY_GRAPH:
            nodes = graph.gateway_nodes
            namespaces = list(set([roslib.names.namespace(n) for n in nodes]))

        elif graph_mode == GATEWAY_PULLED_GRAPH or \
                 graph_mode == GATEWAY_FLIPPED_GRAPH:
            gateway_nodes = graph.gateway_nodes
            connection_nodes = graph.flipped_nodes
            if gateway_nodes or connection_nodes:
                namespaces = [roslib.names.namespace(n) for n in gateway_nodes]
            # an annoyance with the rosgraph library is that it
            # prepends a space to topic names as they have to have
            # different graph node namees from nodes. we have to strip here
            namespaces.extend([roslib.names.namespace(n[1:]) for n in connection_nodes])

        return list(set(namespaces))

    def _filter_orphaned_edges(self, edges, nodes):
        nodenames = [str(n).strip() for n in nodes]
        # currently using and rule as the or rule generates orphan nodes with the current logic
        return [e for e in edges if e.start.strip() in nodenames and e.end.strip() in nodenames]

    def _filter_orphaned_topics(self, connection_nodes, edges):
        '''remove topic graphnodes without connected ROS nodes'''
        removal_nodes = []
        for n in connection_nodes:
            keep = False
            for e in edges:
                if (e.start.strip() == str(n).strip() or e.end.strip() == str(n).strip()):
                    keep = True
                    break
            if not keep:
                removal_nodes.append(n)
        for n in removal_nodes:
            connection_nodes.remove(n)
        return connection_nodes

    def _split_filter_string(self, ns_filter):
        '''splits a string after each comma, and treats tokens with leading dash as exclusions.
        Adds .* as inclusion if no other inclusion option was given'''
        includes = []
        excludes = []
        for name in ns_filter.split(','):
            if name.strip().startswith('-'):
                excludes.append(name.strip()[1:])
            else:
                includes.append(name.strip())
        if includes == [] or includes == ['/'] or includes == ['']:
            includes = ['.*']
        return includes, excludes

    def _get_node_edge_map(self, edges):
        '''returns a dict mapping node name to edge objects partitioned in incoming and outgoing edges'''
        node_connections = {}
        for edge in edges:
            if not edge.start in node_connections:
                node_connections[edge.start] = NodeConnections()
            if not edge.end in node_connections:
                node_connections[edge.end] = NodeConnections()
            node_connections[edge.start].outgoing.append(edge)
            node_connections[edge.end].incoming.append(edge)
        return node_connections

    def _filter_leaves(self,
                            nodes_in,
                            edges_in,
                            node_connections,
                            hide_single_connection_topics,
                            hide_dead_end_topics):
        '''
        removes certain ending topic nodes and their edges from list of nodes and edges

        @param hide_single_connection_topics: if true removes topics that are only published/subscribed by one node
        @param hide_dead_end_topics: if true removes topics having only publishers
        '''
        if not hide_dead_end_topics and not hide_single_connection_topics:
            return nodes_in, edges_in
        # do not manipulate incoming structures
        nodes = copy.copy(nodes_in)
        edges = copy.copy(edges_in)
        removal_nodes = []
        for n in nodes:
            if n in node_connections:
                node_edges = []
                has_out_edges = False
                node_edges.extend(node_connections[n].outgoing)
                if len(node_connections[n].outgoing) > 0:
                    has_out_edges = True
                node_edges.extend(node_connections[n].incoming)
                if ((hide_single_connection_topics and len(node_edges) < 2) or
                    (hide_dead_end_topics and not has_out_edges)):
                    removal_nodes.append(n)
                    for e in node_edges:
                        if e in edges:
                            edges.remove(e)
        for n in removal_nodes:
            nodes.remove(n)
        return nodes, edges

    def generate_dotgraph(self,
                         rosgraphinst,
                         ns_filter,
                         topic_filter,
                         graph_mode,
                         dotcode_factory,
                         show_all_advertisements=False,
                         hide_dead_end_topics=False,
                         cluster_namespaces_level=0,
                         orientation='LR',
                         rank='same',  # None, same, min, max, source, sink
                         ranksep=0.2,  # vertical distance between layers
                         rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
                         simplify=True,  # remove double edges
                         ):
        """
        See generate_dotcode
        """
        includes, excludes = self._split_filter_string(ns_filter)
        connection_includes, connection_excludes = self._split_filter_string(topic_filter)

        gateway_nodes = []
        connection_nodes = []
        # create the node definitions
        if graph_mode == GATEWAY_GATEWAY_GRAPH:
            gateway_nodes = rosgraphinst.gateway_nodes
            gateway_nodes = [n for n in gateway_nodes if matches_any(n, includes) and not matches_any(n, excludes)]
            edges = rosgraphinst.gateway_edges
            edges = [e for e in edges if matches_any(e.label, connection_includes) and not matches_any(e.label, connection_excludes)]

        elif graph_mode == GATEWAY_PULLED_GRAPH or graph_mode == GATEWAY_FLIPPED_GRAPH:
            # create the edge definitions, unwrap EdgeList objects into python lists
            if graph_mode == GATEWAY_PULLED_GRAPH:
                edges = [e for e in rosgraphinst.pulled_edges]
                connection_nodes = rosgraphinst.pulled_nodes
            else:
                edges = [e for e in rosgraphinst.flipped_edges]
                connection_nodes = rosgraphinst.flipped_nodes
            gateway_nodes = rosgraphinst.gateway_nodes
            # filtering the lists
            gateway_nodes = [n for n in gateway_nodes if matches_any(n, includes) and not matches_any(n, excludes)]
            connection_nodes = [n for n in connection_nodes if matches_any(n, connection_includes) and not matches_any(n, connection_excludes)]

        hide_unused_advertisements = not show_all_advertisements
        if graph_mode == GATEWAY_PULLED_GRAPH and hide_unused_advertisements:
            node_connections = self._get_node_edge_map(edges)
            connection_nodes, edges = self._filter_leaves(connection_nodes,
                                         edges,
                                         node_connections,
                                         hide_unused_advertisements,
                                         hide_dead_end_topics)
#        if graph_mode != GATEWAY_GATEWAY_GRAPH and (hide_unused_advertisements or hide_dead_end_topics or accumulate_actions):
#            # maps outgoing and incoming edges to nodes
#            node_connections = self._get_node_edge_map(edges)
#            connection_nodes, edges = self._filter_leaves(connection_nodes,
#                                         edges,
#                                         node_connections,
#                                         hide_unused_advertisements,
#                                         hide_dead_end_topics)

        edges = self._filter_orphaned_edges(edges, list(gateway_nodes) + list(connection_nodes))
        connection_nodes = self._filter_orphaned_topics(connection_nodes, edges)

        # create the graph
        # result = "digraph G {\n  rankdir=%(orientation)s;\n%(nodes_str)s\n%(edges_str)s}\n" % vars()
        dotgraph = dotcode_factory.get_graph(rank=rank,
                                             ranksep=ranksep,
                                             simplify=simplify,
                                             rankdir=orientation)

        namespace_clusters = {}
        for n in (connection_nodes or []):
            # cluster topics with same namespace
            if (cluster_namespaces_level > 0 and
                str(n).count('/') > 1 and
                len(str(n).split('/')[1]) > 0):
                namespace = str(n).split('/')[1]
                if namespace not in namespace_clusters:
                    namespace_clusters[namespace] = dotcode_factory.add_subgraph_to_graph(dotgraph, namespace, rank=rank, rankdir=orientation, simplify=simplify)
                self._add_topic_node(n, dotcode_factory=dotcode_factory, dotgraph=namespace_clusters[namespace])
            else:
                self._add_topic_node(n, dotcode_factory=dotcode_factory, dotgraph=dotgraph)

        # for ROS node, if we have created a namespace clusters for
        # one of its peer topics, drop it into that cluster
        if gateway_nodes is not None:
            for n in gateway_nodes:
                if (cluster_namespaces_level > 0 and
                    str(n).count('/') >= 1 and
                    len(str(n).split('/')[1]) > 0 and
                    str(n).split('/')[1] in namespace_clusters):
                    namespace = str(n).split('/')[1]
                    self._add_node(n, rosgraphinst=rosgraphinst, dotcode_factory=dotcode_factory, dotgraph=namespace_clusters[namespace])
                else:
                    self._add_node(n, rosgraphinst=rosgraphinst, dotcode_factory=dotcode_factory, dotgraph=dotgraph)

        for e in edges:
            self._add_edge(e, dotcode_factory, dotgraph=dotgraph, is_topic=(graph_mode == GATEWAY_GATEWAY_GRAPH))

        return dotgraph

    def generate_dotcode(self,
                         rosgraphinst,
                         ns_filter,
                         topic_filter,
                         graph_mode,
                         dotcode_factory,
                         show_all_advertisements=False,
                         hide_dead_end_topics=False,
                         cluster_namespaces_level=0,
                         orientation='LR',
                         rank='same',  # None, same, min, max, source, sink
                         ranksep=0.2,  # vertical distance between layers
                         rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
                         simplify=True,  # remove double edges
                         ):
        """
        @param rosgraphinst: RosGraph instance
        @param ns_filter: nodename filter
        @type  ns_filter: string
        @param topic_filter: topicname filter
        @type  ns_filter: string
        @param graph_mode str: GATEWAY_GATEWAY_GRAPH | GATEWAY_PULLED_GRAPH | GATEWAY_FLIPPED_GRAPH
        @type  graph_mode: str
        @param orientation: rankdir value (see ORIENTATIONS dict)
        @type  dotcode_factory: object
        @param dotcode_factory: abstract factory manipulating dot language objects
        @param hide_single_connection_topics: if true remove topics with just one connection
        @param hide_dead_end_topics: if true remove topics with publishers only
        @param cluster_namespaces_level: if > 0 places box around members of same namespace (TODO: multiple namespace layers)
        @return: dotcode generated from graph singleton
        @rtype: str
        """
        dotgraph = self.generate_dotgraph(rosgraphinst=rosgraphinst,
                         ns_filter=ns_filter,
                         topic_filter=topic_filter,
                         graph_mode=graph_mode,
                         dotcode_factory=dotcode_factory,
                         show_all_advertisements=show_all_advertisements,
                         hide_dead_end_topics=hide_dead_end_topics,
                         cluster_namespaces_level=cluster_namespaces_level,
                         orientation=orientation,
                         rank=rank,
                         ranksep=ranksep,
                         rankdir=rankdir,
                         simplify=simplify,
                         )
        dotcode = dotcode_factory.create_dot(dotgraph)
        return dotcode
