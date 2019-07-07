:- register_ros_package(knowrob_map_tools).
:- register_ros_package(knowrob_map_data).
:- register_ros_package(knowrob_actions).
:- consult('my_utils').
:- consult('robot').
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_coordinates')).



:- owl_parser:owl_parse('../owl/Robot.owl').



:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(robot, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#', [keep(true)]).

