:- consult('my_utils').

:- module(robot,
    [
        all_times/1,
    ]).

all_times(Time) :- 
    rdf_has(TimeStamp, rdf:type,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#TimeStamp'),
    rdf_has(TimeStamp, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasTime', Time).


 rdf_assert('https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#TimeStamp', 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#isMostRecent', true):-


