/** <module> Predicates for interfacing with the Kautham planner and geometric reasoner

  Copyright (C) 2017 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Daniel Beßler
@license BSD
*/
:- module(knowrob_kautham,
    [
        battat_initialize_kautham_sim/0,
        comp_affordanceocclusion/3,
        kautham_init_planning_scene/2,
        kautham_grab_part/4,
        kautham_put_part/5,
        perform_action/2,
        kautham_planner_run/0
    ]).


:- use_foreign_library('libkauthamwrapper.so').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/beliefstate')).
:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob_assembly')).
:- use_module(library('knowrob_planning')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_kautham, 'http://knowrob.org/kb/knowrob_kautham.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_paramserver, 'http://knowrob.org/kb/knowrob_paramserver.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_toys, 'http://knowrob.org/kb/battat_toys.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_strategy, 'http://knowrob.org/kb/battat_strategy.owl#', [keep(true)]).

:-  rdf_meta
  comp_affordanceocclusion(?, r, r),
  kautham_init_planning_scene(r, r),
  kautham_grab_part(r, r, r, ?),
  kautham_put_part(r, r, r, r, ?).

% calling this will run the planner, and block until its done.
% hopefully, in the mean time the geometric reasoner was invoked, etc.
kautham_planner_run :-
  owl_instance_from_class(battat_toys:'BattatPlaneBodyWithoutWindow', Assemblage),
  agenda_create(Assemblage, battat_strategy:'AgendaStrategy_1', Agenda),
  % this prints the initial planning agenda
  agenda_write(Agenda),
  test_perform_agenda_cram(Agenda).
test_perform_agenda_cram(Agenda) :-
  (agenda_perform_next(Agenda) -> test_perform_agenda_cram(Agenda) ; true).

battat_initialize_kautham_sim :- 
  owl_parser:owl_parse('package://knowrob_kautham/owl/battat_toys.owl'),
  owl_parser:owl_parse('package://knowrob_kautham/owl/battat_strategy_kautham.owl', belief_state).
%%  owl_parser:owl_parse('package://knowrob_kautham/owl/battat_airplane_simulation.owl').

free_grasping_affordance(MobilePart, GraspingAffordance) :-
  owl_has(MobilePart, knowrob:'hasAffordance', GraspingAffordance),
  owl_individual_of(GraspingAffordance, knowrob:'GraspingAffordance'),
  \+ rdf_has(_, knowrob:'blocksAffordance', GraspingAffordance),
  \+ holds(_, knowrob:'blocksAffordance', GraspingAffordance).

grasppose(MobilePart, GraspSpecification, [PTx, PTy, PTz, PRx, PRy, PRz, PRw]) :-
  belief_at_global(MobilePart, [_, _, [OTx, OTy, OTz], [ORx, ORy, ORz, ORw]]),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
  transform_multiply(["map", "mobilepart", [OTx, OTy, OTz], [ORx, ORy, ORz, ORw]], ["mobilepart", "gripper", [GTx, GTy, GTz], [GRx, GRy, GRz, GRw]], [_, _, [PTx, PTy, PTz], [PRx, PRy, PRz, PRw]]).

arm_available([PTx, PTy, PTz, PRx, PRy, PRz, PRw], "left") :-
  kautham_call_yumi_ik("left", [PTx, PTy, PTz, PRx, PRy, PRz, PRw], Conf),
  \+ =([], Conf).

arm_available([PTx, PTy, PTz, PRx, PRy, PRz, PRw], "right") :-
  kautham_call_yumi_ik("right", [PTx, PTy, PTz, PRx, PRy, PRz, PRw], Conf),
  \+ =([], Conf).

get_connection_transform(Connection, [CTx, CTy, CTz, CRx, CRy, CRz, CRw], ConnectionTransform, ReferenceObj) :-
  once(owl_has(Connection, knowrob_assembly:'usesTransform', ConnectionTransform)),
  assemblage_connection_reference(Connection, ConnectionTransform, ReferenceObj), 
  rdf_has(ConnectionTransform, knowrob:'translation', literal(type(_, ConnectionTranslation))),
  rdf_has(ConnectionTransform, knowrob:'quaternion', literal(type(_, ConnectionRotation))),
  rdf_vector_prolog(ConnectionTranslation, [CTx, CTy, CTz]),
  rdf_vector_prolog(ConnectionRotation, [CRx, CRy, CRz, CRw]).

put_away_manips(GraspingAffordance, GraspSpecification, PartGlobalTargetPose, ArmName, Result) :-
  kautham_grab_part(GraspingAffordance, GraspSpecification, ArmName, GrabResult),
  (=('ok', GrabResult) -> 
    kautham_put_part(GraspingAffordance, GraspSpecification, PartGlobalTargetPose, ArmName, Result);
    =(GrabResult, Result)).

get_pose_and_dist([Tx, Ty, Tz, Rx, Ry, Rz, Rw, D]) :-
  owl_individual_of(Part, knowrob_assembly:'AtomicPart'),
  belief_at_global(Part, [_, _, [Tx, Ty, Tz], [Rx, Ry, Rz, Rw]]),
  %%%%%% Add dist to the rep.
  =(D, 0.15).

get_poses_and_dists(PartPosesAndDists) :-
  findall(Pd, get_pose_and_dist(Pd), PartPosesAndDists).

perform_action(ActionDescription, Result) :-
%% perform_put_away(ActionDescription, Result) :-
  %% =(ActionDescription, ["an", "action", ["type", "putting_part_away"], ["mobile-part", MobilePart]]),
  rdfs_type_of(ActionDescription, knowrob_assembly:'PutAwayPart'),!,
  writeln('Kautham perform PutAwayPart...'),
  % TODO: do not ignore knowrob:'avoidedObject'
  rdf_has(ActionDescription, knowrob:'movedObject', MobilePart),
  free_grasping_affordance(MobilePart, GraspingAffordance),
  rdf_has(GraspingAffordance, knowrob_assembly:'graspAt', GraspSpecification),
  grasppose(MobilePart, GraspSpecification, [PTx, PTy, PTz, PRx, PRy, PRz, PRw]),
  arm_available([PTx, PTy, PTz, PRx, PRy, PRz, PRw], ArmName),
  %%%%%% Define table surface limits [fwd back left right height] and object height;
  =(TableLimits, [0.6, 0, -0.5, 0.5, 0]),
  =(PartHeight, 0.1),
  get_poses_and_dists(PartPosesAndDists),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
  kautham_find_away_pose(ArmName, TableLimits, PartHeight, [GTx, GTy, GTz, GRx, GRy, GRz, GRw], [PRx, PRy, PRz, PRw], PartPosesAndDists, PartGlobalTargetPose, FindResult),
  ((=('ok', FindResult) ->
    put_away_manips(GraspingAffordance, GraspSpecification, PartGlobalTargetPose, ArmName, Result);
    =(FindResult, Result))),
  =('ok', Result).

perform_action(ActionDescription, Result) :-
%% perform_assembly_action(ActionDescription, Result) :-
  %% =(ActionDescription, ["an", "action", ["type", "connecting"], ["connection", Connection], ["fixed-part", _], ["mobile-part", MobilePart]]),
  rdfs_type_of(ActionDescription, knowrob_assembly:'ConnectingParts'), !,
  writeln('Kautham perform ConnectingParts...'),
  rdf_has(ActionDescription, knowrob_assembly:'mobilePart', MobilePart),
  rdf_has(ActionDescription, knowrob_assembly:'assembledConnection', Connection),
  assemblage_possible_grasp(MobilePart, Connection, [GraspPart, GraspingAffordance, GraspSpecification]),
  grasppose(GraspPart, GraspSpecification, [PTx, PTy, PTz, PRx, PRy, PRz, PRw]),
  arm_available([PTx, PTy, PTz, PRx, PRy, PRz, PRw], ArmName),
  get_connection_transform(Connection, [CTx, CTy, CTz, CRx, CRy, CRz, CRw], TransformId, ReferenceObj),
  belief_at_global(ReferenceObj, [_, _, [OTx, OTy, OTz], [ORx, ORy, ORz, ORw]]),
  transform_multiply(["map", "fixedpart", [OTx, OTy, OTz], [ORx, ORy, ORz, ORw]], ["fixedpart", "mobilepart", [CTx, CTy, CTz], [CRx, CRy, CRz, CRw]], [_, _, [PTx, PTy, PTz], [PRx, PRy, PRz, PRw]]),
  =(PartGlobalTargetPose, [[PTx, PTy, PTz], [PRx, PRy, PRz, PRw]]),
  kautham_grab_part(GraspingAffordance, GraspSpecification, ArmName, GrabResult),
  ((=('ok', GrabResult) -> 
    kautham_put_part(GraspingAffordance, GraspSpecification, PartGlobalTargetPose, ArmName, Result);
    =(GrabResult, Result))),
  =('ok', Result).

get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]) :-
  rdf_has(GraspSpecification, knowrob_paramserver:'hasGraspTransform', GraspTransform),
  rdf_has(GraspTransform, knowrob:'translation', literal(type(_, GraspTranslation))),
  rdf_has(GraspTransform, knowrob:'quaternion', literal(type(_, GraspRotation))),
  rdf_vector_prolog(GraspTranslation, [GTx, GTy, GTz]),
  rdf_vector_prolog(GraspRotation, [GRx, GRy, GRz, GRw]).

%% comp_affordanceocclusion(?Part, +Affordance)
comp_affordanceocclusion(Part, GraspingAffordance) :-
  writeln('comp_affordanceocclusion/2 called'),
  fail.

%% comp_affordanceocclusion(?Part, +Affordance, +ArmName)
%
comp_affordanceocclusion(Part, GraspingAffordance, ArmName) :-
% Retrieve transform associated to GraspingAffordance
  rdf_has(GraspingAffordance, knowrob_assembly:'graspAt', GraspSpecification),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
% Retrieve TargetPart (associated to GraspingAffordance)
  rdf_has(TargetPart, knowrob:'hasAffordance', GraspingAffordance),
% Retrieve pose associated to TargetPart
  belief_at_global(TargetPart, [_, _, [OTx, OTy, OTz], [ORx, ORy, ORz, ORw]]),
% Call helper program: give transform to TargetPart, return a list of colliding bodies (represented as indices in planning scene)
  kautham_blocking_objects([OTx, OTy, OTz, ORx, ORy, ORz, ORw], [GTx, GTy, GTz, GRx, GRy, GRz, GRw], CollidingObjectIndices, ArmName),
% retrieve Part so that it has planningSceneIndex in the list
  member(Index, CollidingObjectIndices),
  rdf_has(Part, knowrob_kautham:'planningSceneIndex', literal(type(_, Index))).

part_data(Part, Mesh, Pose) :-
  object_mesh_path(Part, Mesh),
  belief_at_global(Part, [_, _, [Tx, Ty, Tz], [Rx, Ry, Rz, Rw]]),
  =(Pose, [Tx, Ty, Tz, Rx, Ry, Rz, Rw]).

assert_part_index([Part, Index]) :-
  rdf_assert(Part, knowrob_kautham:'planningSceneIndex', literal(type('http://www.w3.org/2001/XMLSchema#integer', Index))).

kautham_init_planning_scene(ModelFolder, SceneMap) :-
  kautham_init_planning_scene_internal(ModelFolder, SceneMap),
  findall(Part, (owl_individual_of(Part, knowrob_assembly:'AtomicPart')), PartList),
  list_to_set(PartList, Parts),
  findall([Part, Mesh, Pose], (member(Part, Parts), part_data(Part, Mesh, Pose)), PartDataList),
  list_to_set(PartDataList, PartData),
  kautham_add_obstacles_internal(PartData, Indices),
  maplist(assert_part_index, Indices).

kautham_grab_part(GraspingAffordance, GraspSpecification, ArmName, Result) :-
  (=('left', ArmName) ->
    =('http://knowrob.org/kb/battat_airplane_simulation.owl#LeftYumiGripper', Gripper) ; =('http://knowrob.org/kb/battat_airplane_simulation.owl#RightYumiGripper', Gripper)),
  rdf_has(Part, knowrob:'hasAffordance', GraspingAffordance),
  belief_at_global(Part, [_, _, [TTx, TTy, TTz], [TRx, TRy, TRz, TRw]]),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
  rdf_has(Part, knowrob_kautham:'planningSceneIndex', literal(type(_, ObjectIndex))),
  once(kautham_grab_part_internal([TTx, TTy, TTz, TRx, TRy, TRz, TRw], [GTx, GTy, GTz, GRx, GRy, GRz, GRw], ObjectIndex, ArmName, Result)), !,
  (=('ok', Result) ->
    once(kautham_assembly_apply_grasp(Part, Gripper, GraspSpecification)),
    true
   ; true).

kautham_put_part(GraspingAffordance, GraspSpecification, PartGlobalTargetPose, ArmName, Result) :-
  (=('left', ArmName) ->
    =('http://knowrob.org/kb/battat_airplane_simulation.owl#LeftYumiGripper', Gripper) ; =('http://knowrob.org/kb/battat_airplane_simulation.owl#RightYumiGripper', Gripper)),
  rdf_has(Part, knowrob:'hasAffordance', GraspingAffordance),
  =(PartGlobalTargetPose, [[TTx, TTy, TTz], [TRx, TRy, TRz, TRw]]),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
  rdf_has(Part, knowrob_kautham:'planningSceneIndex', literal(type(_, ObjectIndex))),
  once(kautham_put_part_internal([TTx, TTy, TTz, TRx, TRy, TRz, TRw], [GTx, GTy, GTz, GRx, GRy, GRz, GRw], ObjectIndex, ArmName, Result)), !,
  (=('ok', Result) ->
    kautham_assembly_apply_ungrasp(Part, Gripper, GraspSpecification) ; true).

kautham_assembly_apply_grasp(GraspedObject, Gripper, GraspSpec) :-
  format("APPLY-GRASP~n"),
  ground(GraspedObject), ground(Gripper), 
  format(" ... grounded~n    ~a~n    ~a~n", [GraspedObject, Gripper]),
  once((
    rdf_has(GraspedObject, knowrob:'hasAffordance', GraspedAffordance),
    owl_has(GraspedAffordance, knowrob_assembly:'graspAt', GraspSpec)
  )),
  format("    ~a~n", [GraspSpec]),
  assemblage_mechanical_part(GraspedObject),
  rdf_has(Gripper, knowrob:'frameName', literal(_)),
  format(" passed some checks~n"),
  %%%%
  rdf_has(GraspSpec, knowrob_paramserver:'hasGraspTransform', TransformId),
  transform_data(TransformId, TransformData),
  % retract connections to fixed objects
  kautham_get_global_pose(GraspedObject, ObjectMapData),
  %assemblage_remove_fixtures(GraspedObject),
  % there could be objects with transforms not connected to GraspedObject.
  % find the physical bridges to tf unconnected objects and make
  % tf unconnected objects transforms relative to the bridge.
  findall(X, assemblage_part_connect_transforms(GraspedObject, X), DirtyUnconnected),
  format("    ~w~n", [DirtyUnconnected]),
  % invert the transform topology along the parent frame relation.
  % GraspedObject is then reference of all phsically connected parts
  assemblage_part_make_reference(GraspedObject, Parents),
  format(" Parents from make reference~n    ~w~n", [Parents]),
  % apply grasp transform on grasped object
  belief_at_internal(GraspedObject, TransformData, Gripper),
  % accumulate list of dirty objects and cause beliefstate to republish TF frames
  findall(X, ( member(X, [GraspedObject|Parents]) ;
    ( member(List, DirtyUnconnected), member(X,List) )), Dirty),
  format(" Dirty~n    ~w~n", [Dirty]),
  belief_republish_objects(Dirty), !.
  % assert temporary connections that consume affordances blocked by the grasp
  %kautham_assembly_block_grasp_affordances(GraspedAffordance).

kautham_assembly_apply_ungrasp(GraspedObject, Gripper, GraspSpec) :-
  %%%% input checking
  ground(GraspedObject), ground(Gripper), ground(GraspSpec),
  once((
    rdf_has(GraspedObject, knowrob:'hasAffordance', GraspedAffordance),
    owl_has(GraspedAffordance, knowrob_assembly:'graspAt', GraspSpec)
  )),
  assemblage_mechanical_part(GraspedObject),
  rdf_has(Gripper, knowrob:'frameName', literal(_)),
  format("APPLY-UNGRASP~n"),
  format(" ... grounded~n    ~a~n    ~a~n", [GraspedObject, Gripper]),
  format("    ~a~n", [GraspSpec]),
  %%%%
  % make GraspedObject absolute if still relative to gripper
  rdf_has(GraspedObject, knowrob:'pose', TransformId),
  ( rdf_has(TransformId, knowrob:'relativeTo', Gripper) -> (
  sleep(1.0),
    kautham_get_global_pose(GraspedObject, [[Tx, Ty, Tz], [Rx, Ry, Rz, Rw]]),
    belief_at_update(GraspedObject, ([Tx,Ty,Tz], [Rx,Ry,Rz,Rw]))
  ) ; true ), !.
  % retract temporary connections that consume affordances blocked by the grasp
  %kautham_assembly_unblock_grasp_affordances(GraspedAffordance).

kautham_assembly_block_grasp_affordances(GraspedAffordance) :-
  % block the affordance that is grasped
  rdf_instance_from_class(knowrob_assembly:'GraspingConnection', GraspConnection),
  format(" New individual ~a~n", [GraspConnection]),
  rdf_assert(GraspConnection, knowrob_assembly:'consumesAffordance', GraspedAffordance).

kautham_assembly_unblock_grasp_affordances(GraspedAffordance) :-
  rdf_has(GraspConnection, knowrob_assembly:'consumesAffordance', GraspedAffordance),
  rdfs_individual_of(GraspConnection, knowrob_assembly:'GraspingConnection'),
  rdf_retractall(GraspConnection, _, _).

