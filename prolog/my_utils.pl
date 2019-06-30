:- module(my_utils,
    [
      rdf_costom_instance_from_class/4
      is_in_turn/2
    ]).



%%%%%%%%% costom instance

rdf_costom_instance_from_class(Class, SourceRef,Instance_ID, Instance) :-
  % create instance from type
  ((concat_atom(List, '#', Class),length(List,Length),Length>1) -> (
    % Class is already a URI
    T=Class
  );(
    atom_concat('http://knowrob.org/kb/knowrob.owl#', Class, T)
  )),
  atom_concat(Class,  '_', Class2),
  atom_concat(Class2, Instance_ID, Instance),

  ( ( nonvar(SourceRef), rdf_assert(Instance, rdf:type, T, SourceRef),!);
    ( rdf_assert(Instance, rdf:type, T)) 
).

% win_game(A):-length(CardCollection,num),

% win_cards(Player, Cards):-

is_in_turn(Player,Turn):- rdf_assert(Player,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasCurrentTurn',Turn).

