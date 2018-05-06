
:- module(knowrob_battat,
    [
      battat_init_scene/1,
      %%
      test_perform/2,
      test_comp_occludedBy/2,
      
        battat_initialize/0,
        battat_initialize_sim/0,
        battat_sim_plane_complete/0,
        battat_sim_plane_connection/3
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).

:- rdf_db:rdf_register_prefix(battat_toys, 'http://knowrob.org/kb/battat_toys.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_test, 'http://knowrob.org/kb/battat_airplane_test.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_sim, 'http://knowrob.org/kb/battat_simulation.owl#', [keep(true)]).

:-  rdf_meta
      battat_sim_plane_connection(r,r,t),
      battat_sim_plane_connection(r,r,t,r).

battat_initialize :-
  owl_parser:owl_parse('package://knowrob_assembly/owl/battat_toys.owl'),
  owl_parser:owl_parse('package://knowrob_assembly/owl/battat_strategy.owl', belief_state),
  owl_parser:owl_parse('package://knowrob_srdl/owl/Boxy_08_2016.owl').

battat_initialize_sim :-
  battat_initialize,
  owl_parser:owl_parse('package://knowrob_assembly/owl/battat_airplane_simulation.owl').

battat_init_scene(SceneOntology) :-
  owl_parser:owl_parse('package://knowrob_assembly/owl/battat_toys.owl'),
  owl_parser:owl_parse('package://knowrob_assembly/owl/battat_strategy.owl', belief_state),
  owl_parser:owl_parse(SceneOntology, belief_state).

battat_sim_plane_complete :-
  battat_sim_plane_connection(battat_toys:'BottomWingSlideInChassis', battat_sim:'PlaneBottomWing_1',
     [battat_sim:'PlaneChassis_1',battat_sim:'PlaneBottomWing_1']),
  battat_sim_plane_connection(battat_toys:'UnderBodySlideInFrame', battat_sim:'PlaneUnderBody_1',
      [battat_sim:'PlaneUnderBody_1',battat_sim:'PlaneChassis_1',battat_sim:'PlaneRearWing_1',battat_sim:'PlaneBottomWing_1']),
  battat_sim_plane_connection(battat_toys:'MotorGrillSlideInUnderBody', battat_sim:'PlaneMotorGrill_1',
      [battat_sim:'PlaneUnderBody_1',battat_sim:'PlaneMotorGrill_1']),
  battat_sim_plane_connection(battat_toys:'UpperBodySlideInFrame', battat_sim:'PlaneUpperBody_1',
      [battat_sim:'PlaneUpperBody_1',battat_sim:'PlaneUnderBody_1',battat_sim:'PlaneRearWing_1',battat_sim:'PlaneMotorGrill_1']),
  battat_sim_plane_connection(battat_toys:'TopWingSlideInBody', battat_sim:'PlaneTopWing_1',
      [battat_sim:'PlaneUpperBody_1',battat_sim:'PlaneTopWing_1']),
  battat_sim_plane_connection(battat_toys:'PropellerSlideInMotorGrill', battat_sim:'PlanePropeller_1',
      [battat_sim:'PlaneMotorGrill_1',battat_sim:'PlanePropeller_1']),
  battat_sim_plane_connection(battat_toys:'WindowSlideInBody', battat_sim:'PlaneWindow_1',
      [battat_sim:'PlaneUpperBody_1',battat_sim:'PlaneTopWing_1',battat_sim:'PlaneWindow_1']),
  %%% Front wheels
  battat_sim_plane_connection(battat_toys:'FrontWheelLeftSlideInChassis', battat_sim:'PlaneFrontWheel_1',
      [battat_sim:'PlaneChassis_1',battat_sim:'PlaneFrontWheel_1']),
  battat_sim_plane_connection(battat_toys:'FrontWheelRightSlideInChassis', battat_sim:'PlaneFrontWheel_2',
      [battat_sim:'PlaneChassis_1',battat_sim:'PlaneFrontWheel_2']),
  %% Nuts
  battat_sim_plane_connection(battat_toys:'PlaneFrontWheelLeftScrewedOnChassis', battat_sim:'BattatNut_1',
      [battat_sim:'PlaneChassis_1',battat_sim:'BattatNut_1']),
  battat_sim_plane_connection(battat_toys:'PlaneFrontWheelLeftScrewedOnChassis', battat_sim:'BattatNut_2',
      [battat_sim:'PlaneChassis_1',battat_sim:'BattatNut_2']),
  %% Bolts
  battat_sim_plane_connection(battat_toys:'FrontWingsScrewedOnBody', battat_sim:'BattatBolt_2',
      [battat_sim:'PlaneUpperBody_1',battat_sim:'PlaneTopWing_1',battat_sim:'BattatBolt_2']),
  battat_sim_plane_connection(battat_toys:'PlaneWindowScrewedOnBody', battat_sim:'BattatBolt_3',
      [battat_sim:'PlaneUpperBody_1',battat_sim:'PlaneWindow_1',battat_sim:'BattatBolt_3']),
  battat_sim_plane_connection(battat_toys:'PropellerScrewedOnMotorGrill', battat_sim:'BattatBolt_4',
      [battat_sim:'PlanePropeller_1',battat_sim:'PlaneMotorGrill_1',battat_sim:'BattatBolt_4']),
  battat_sim_plane_connection(battat_toys:'RearWingScrewedOnBody', battat_sim:'BattatBolt_5',
      [battat_sim:'PlaneUpperBody_1',battat_sim:'PlaneRearWing_1',battat_sim:'BattatBolt_5']).

battat_sim_plane_connection(ConnType, Primary, Parts) :- battat_sim_plane_connection(ConnType, Primary, Parts, _).
battat_sim_plane_connection(ConnType, Primary, Parts, Conn) :-
  assemblage_connection_create(ConnType, Parts, Conn),
  write('  connection '), owl_write_readable(Conn), nl,
  write('    primary: '), owl_write_readable(Primary), nl,
  write('    parts:   '), owl_write_readable(Parts), nl,
  forall( rdf_has(Conn, knowrob_assembly:'blocksAffordance', Aff), (
          write('    blocksAffordance '), owl_write_readable(Aff), nl )),
  cram_assembly:cram_assembly_apply_connection(Primary, Conn).

:- dynamic put_away_performed/2.

test_comp_occludedBy(A,B) :-
  rdf_equal(A,battat_sim:'PlaneBottomWing_1'),
  rdf_equal(B,battat_sim:'PlaneUpperBody_1'),
  \+ put_away_performed(A,B).

test_perform(Act,_) :-
  rdfs_individual_of(Act,knowrob_assembly:'PutAwayPart'), !,
  test_perform_put_away(Act).
test_perform(Act,_) :-
  rdfs_individual_of(Act,knowrob_assembly:'ConnectingParts'), !,
  test_perform_connecting_parts(Act).

test_perform_connecting_parts(PutAway) :-
  writeln('PERFORM ConnectingParts'),
  write_action__(PutAway).

test_perform_put_away(PutAway) :-
  writeln('PERFORM PutAwayPart'),
  write_action__(PutAway),
  %%%
  rdf_equal(battat_sim:'PlaneBottomWing_1',A),
  rdf_equal(battat_sim:'PlaneUpperBody_1',B),
  rdf_has(PutAway, knowrob:avoidedObject, A),
  rdf_has(PutAway, knowrob:movedObject, B),
  assertz(put_away_performed(A,B)), !.
test_perform_put_away(_).

write_action__(Act) :-
  entity(Act,Descr),
  entity_write(Descr), !.
