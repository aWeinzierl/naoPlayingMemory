:- module(my_utils,
    [
      rdf_costom_instance_from_class/4
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

