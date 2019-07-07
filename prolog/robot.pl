:- module(robot,
    [
        all_times/1,
        most_recent_timestamp/1,
        largest/2,
        start_game/1,
        create_turn/3
    ]).

all_times(Time) :- 
    rdfs_individual_of(TimeInstances,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#TimeStamp'),
    rdf_has(TimeInstances, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasTime', Time).
    

most_recent_timestamp(Vtime) :-
    findall(Times,all_times(Times), List),
    maplist(atom_number,List,List_n),
    largest(List_n,Vtime).

largest([X],X).
largest([X|Xs],R):-
   largest(Xs,Y),
   R is max(X,Y).

create_turn(Turn, Player, TimeN):-
    rdf_costom_instance_from_class('https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Turn',_, TimeN, Turn),
    rdf_assert(Turn, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#associatedToPlayer', Player).


start_game(GameStatus):- 
    rdfs_individual_of(InstanceStartGame,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#StartGame'),
    rdf_has(InstanceStartGame,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasTimeStamp',TimeStartGame),
    rdf_has(TimeStartGame,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasTime',Time),
    atom_number(Time,TimeN),
    rdf_costom_instance_from_class('https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#MemoryGame',_,TimeN,MemoryGame),
    rdf_costom_instance_from_class('https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#MemoryGameStatus',_,TimeN,GameStatus),
    rdf_costom_instance_from_class('https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Round',_, TimeN, Round),
    create_turn(Turn, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#1', TimeN),
    rdf_assert(MemoryGame,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasStatus',GameStatus),
    rdf_assert(GameStatus,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasPlayer','https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#1'),
    rdf_assert(GameStatus,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasPlayer','https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#2'),
    rdf_assert(GameStatus,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#currentRound',Round),
    rdf_assert(Round,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasCurrebtTurn',Turn).


    








