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
        findTwoEqualCards/2,
        findTwoEqualCards_pos/5,
        findTwoEqualCards_with_id/4,
        act/2,
        act_id/2,
        act_id_pos/6,
        pickRandomCard/1,
        pickRandomCard_Class_Or_Without/1,
        pickRandomCard_pos/3,
        delete_cards/2,
        check_deletion/1,
        giveClass_pos/3,
        %unknownCardInstanciation/2,
        %knownCardInstanciation/2,
        findPosition/3
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
    not(findTwoEqualCards(_,_)), % not(findTwoEqualCards(C1,C2)),
    not(rdf_has(Card, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass', _)), % _ =Class
    hasZeroAttempts(Player).
    
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


% Important Query!! Decision1
findTwoEqualCards(C1, C2):-
    rdfs_individual_of(C1, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    rdfs_individual_of(C2, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    rdf_has(C1, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id1),
    rdf_has(C2, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id2),
    atom_number(Id1,Id1N),atom_number(Id2,Id2N),
    ((Id1N<Id2N);(Id2N<Id1N)),
    rdf_has(C1, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass', Class),
    rdf_has(C2, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass', Class),
    not(check_deletion(Id1)),
    not(check_deletion(Id2)).



pickRandomCard(Card):- 
    rdfs_individual_of(Card, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    not(rdf_has(Card, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass', Class)),
    rdf_has(Card, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id1),
    not(check_deletion(Id1)).

findTwoEqualCards_pos(C1, C2,Id2,C2X,C2Y):-
    rdfs_individual_of(C1, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    rdfs_individual_of(C2, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    rdf_has(C1, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id1),
    rdf_has(C2, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id2),
    atom_number(Id1,Id1N),atom_number(Id2,Id2N),
    ((Id1N<Id2N);(Id2N<Id1N)),
    rdf_has(C1, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass', Class),
    rdf_has(C2, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass', Class),
    findPosition(C2X,C2Y,C2),
    not(check_deletion(Id1)),
    not(check_deletion(Id2)).

findTwoEqualCards_with_id(C1id,C2id,C1,C2):-
    rdfs_individual_of(C1,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    rdfs_individual_of(C2,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    rdf_has(C1,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',C1id),
    rdf_has(C2,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',C2id),
    findTwoEqualCards(C1,C2).



act(C1,C2):-
    findTwoEqualCards(C1,C2);
    pickRandomCard(C1).


act_id(C1id,C2id):-
    (
        findTwoEqualCards(C1,C2),
        rdf_has(C1,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',C1id),
        rdf_has(C2,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',C2id)
    );(
        pickRandomCard(C1),
        rdf_has(C1,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',C1id)
    ).


act_id_pos(C1id,C1X,C1Y,C2id,C2X,C2Y):-
    (
        findTwoEqualCards(C1,C2),
        rdf_has(C1,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',C1id),
        findPosition(C1X,C1Y,C1),
        rdf_has(C2,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',C2id),
        findPosition(C2X,C2Y,C2)
    );(
        pickRandomCard(C1),
        rdf_has(C1,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',C1id),
        findPosition(C1X,C1Y,C1)
    ).


findPosition(X,Y,Card):-
    rdf_has(Card,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasPosition',CardPosition),
    rdf_has(CardPosition,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasXCoordinate',X),
    rdf_has(CardPosition,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasYCoordinate',Y).
    



/*hasPair(Card):-
    rdfs_individual_of(Card,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    rdfs_individual_of(Card2,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    rdf_has(Card, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass', Class),
    rdf_has(Card2, 'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass', Class).
*/


pickRandomCard_Class_Or_Without(Card):-
    (
        pickRandomCard(Card)
    );(
        rdfs_individual_of(Card,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
        rdf_has(Card,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id1),
        not(check_deletion(Id1)),
        not(findTwoEqualCards(Card,Card2))
    ).

pickRandomCard_pos(C1,C1X,C1Y):-
    pickRandomCard(C1),
    findPosition(C1X,C1Y,C1).

/*delete_cards(Id1,Id2):-
    rdf_has(C1,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id1),
    rdf_has(C2,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id2),
    retract(C1),
    retract(C2).*/

/*
unknownCardInstanciation(ardPositionName,CardPositionX ,CardPositionY,MarkerId,CardInstance):-
    rdf_costom_instance_from_class('https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card',_,MarkerId,CardInstance),
    rdf_assert(CardInstance,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',MarkerId),
    rdf_costom_instance_from_class('https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#CardPosition',_,CardPositionName,CardPositionInstance),
    rdf_assert(CardPositionInstance,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasXCoordinate',CardPositionX),
    rdf_assert(CardPositionInstance,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasYCoordinate',CardPositionY),
    rdf_assert(CardInstance,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasPosition',CardPositionInstance).*/



knownCardInstanciation(UnknownCardInstance,Class):-
    most_recent_time_stamp(TimeStamp),
    rdf_assert(CardInstance,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card',TimeStamp),
    rdf_assert(UnknownCardInstance,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasYCoordinate',Class).

    
    
delete_cards(Id1,Id2):-
    rdf_has(C1,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id1),
    rdf_assert(C1,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#isDeleted',true), 
    rdf_has(C2,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id2),
    rdf_assert(C2,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#isDeleted',true).

check_deletion(Id1):-
    rdf_has(C1,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasMarkerId',Id1),
    rdf_has(C1,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#isDeleted',true).

giveClass_pos(CX,CY,Class):-
    rdfs_individual_of(Card,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card'),
    rdf_has(Card,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasPosition',CardPosition),
    rdf_has(CardPosition,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasXCoordinate',CX),
    rdf_has(CardPosition,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasYCoordinate',CY),
    rdf_assert(Card,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass',Class).


    
/*reset_prolog(C):-
    rdf_assert(C,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#isDeleted',false).*/




    
   
 

    



    


