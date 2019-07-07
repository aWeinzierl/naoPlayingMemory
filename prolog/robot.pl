:- module(robot,
    [
        all_times/1,
        most_recent_timestamp/1,
        largest/2
    ]).

all_times(Time) :- 
    rdfs_individual_of(TimeInstances,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#TimeStamp'),
    rdf_has(TimeInstances, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasTime', Time).
    

most_recent_timestamp(Vtime) :-
    findall(Times,all_times(Times), List),
    maplist(atom_number,List,List_n),
    largest(List_n,Vtime).

largest([X],X).
largest([X|Xs],R) :-
   largest(Xs,Y),
   R is max(X,Y).
   

