:- module(robot,
    [
        all_times/1,
        most_recent_time/1,
        most_recent_time_stamp/1,
        largest/2,
        start_game/1,
        create_turn/3,
        canPlayAttempt/3,
        hasTurn/2,
        hasZeroAttempts/1,
        findTwoEqualCards/2
    ]).

all_times(Time) :- 
    rdfs_individual_of(TimeInstances,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#TimeStamp'),
    rdf_has(TimeInstances, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasTime', Time).
    

most_recent_time(Vtime) :-
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
    rdf_assert(GameStatus,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasTimeStamp',TimeStartGame),  
    rdf_assert(GameStatus,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasPlayer','https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#1'),
    rdf_assert(GameStatus,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasPlayer','https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#2'),
    rdf_assert(GameStatus,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#currentRound',Round),
    rdf_assert(Round,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasCurrentTurn',Turn),
    rdf_assert(Turn, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#associatedToPlayer', 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#1').

canPlayAttempt(Player,  'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#turnOneCard', Card):- 
    rdfs_individual_of(Card, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    not(findTwoEqualCards(C1,C2)),
    not(rdf_has(Card, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass', Class)),
    hasZeroAttempts(Player).
    #most_recent_time(Time),
    #rdf_costom_instance_from_class('https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Attempt',_, Time, Attempt),
    #hasTurn(Player,Turn),
    #rdf_assert(Turn, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasCurrentAttempt', Attempt),
    #rdf_assert(Attempt, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasAction', 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#turnOneCard').

most_recent_time_stamp(TimeStamp):-
    most_recent_time(TimeN),
    atom_number(Time,TimeN),
    rdf_has(TimeStamp, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasTime', Time).

hasTurn(Player, Turn) :-
    most_recent_time_stamp(TimeStamp),
    rdfs_individual_of(GameStatus, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#MemoryGameStatus'),
    rdf_has(GameStatus,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasTimeStamp',TimeStamp),
    rdf_has(GameStatus, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#currentRound', Round),
    rdf_has(Round, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasCurrentTurn', Turn),
    rdf_has(Turn,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#associatedToPlayer', Player).
    

hasZeroAttempts(Player):-
    hasTurn(Player, Turn),
    not(rdf_has(Turn, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasCurrentAttempt', _)).

findTwoEqualCards(C1, C2):-
    rdfs_individual_of(C1, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    rdfs_individual_of(C2, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    rdf_has(C1, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id1),
    rdf_has(C2, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id2),
    atom_number(Id1,Id1N),atom_number(Id2,Id2N),
    (Id1N<Id2N),
    rdf_has(C1, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass', Class1),
    rdf_has(C2, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass', Class2).

UpdateGameStatusTurnCardAction(Action):-
    most_recent_time_stamp(TimeStamp),
    CopyGameStatus().


    
   
 

    



    


