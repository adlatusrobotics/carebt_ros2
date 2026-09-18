# Copyright 2026 Adlatus Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for CareBT execution tracing."""

import importlib.util
from pathlib import Path
import sys

from carebt import ActionNode, BehaviorTreeRunner, NodeStatus, SequenceNode


MODULE_PATH = Path(__file__).parents[1] / 'carebt_ros2' / 'execution_trace.py'
SPEC = importlib.util.spec_from_file_location('execution_trace_under_test', MODULE_PATH)
execution_trace = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = execution_trace
SPEC.loader.exec_module(execution_trace)

CarebtExecutionTraceHooks = execution_trace.CarebtExecutionTraceHooks
ExecutionTraceConfig = execution_trace.ExecutionTraceConfig
ExecutionTraceRecorder = execution_trace.ExecutionTraceRecorder
ExecutionTraceSerializer = execution_trace.ExecutionTraceSerializer
TraceEventType = execution_trace.TraceEventType
TraceLifecycle = execution_trace.TraceLifecycle


class AddOne(ActionNode):
    """Return the input value incremented by one."""

    def __init__(self, runner):
        super().__init__(runner, '?value => ?result')

    def on_tick(self):
        self._result = self._value + 1
        self.set_status(NodeStatus.SUCCESS)


class ParameterSequence(SequenceNode):
    """Execute AddOne with parent member bindings."""

    def __init__(self, runner):
        super().__init__(runner)

    def on_init(self):
        self.value = 2
        self.result = None
        self.append_child(AddOne, 'value => result')


class PendingSequence(SequenceNode):
    """Declare a child without executing it."""

    def __init__(self, runner):
        super().__init__(runner)

    def on_init(self):
        self.append_child(AddOne, '1 => unused')


def test_records_topology_lifecycle_and_declared_parameters():
    """A completed sequence retains topology and latest declared I/O."""
    runner = BehaviorTreeRunner()
    recorder = ExecutionTraceRecorder()
    assert CarebtExecutionTraceHooks.install() == ''
    CarebtExecutionTraceHooks.register(runner, recorder)
    try:
        recorder.start_session()
        runner.run(ParameterSequence)
        recorder.end_session()
    finally:
        CarebtExecutionTraceHooks.unregister(runner)

    snapshot = recorder.snapshot()
    assert [node.class_name for node in snapshot.nodes] == [
        '_RootNode', 'ParameterSequence', 'AddOne']
    assert [node.parent_call_id for node in snapshot.nodes] == [0, 1, 2]
    assert all(node.lifecycle == TraceLifecycle.COMPLETED for node in snapshot.nodes)
    assert recorder.snapshot(include_completed=False).nodes == ()

    parameters = [
        parameter
        for node in snapshot.nodes
        for parameter in node.parameters.values()
    ]
    assert any(
        parameter.name == 'value'
        and parameter.binding == 'value'
        and parameter.value == '2'
        for parameter in parameters
    )
    assert any(
        parameter.name == 'result'
        and parameter.binding == 'result'
        and parameter.value == '3'
        for parameter in parameters
    )
    add_one_call_id = snapshot.nodes[-1].call_id
    assert any(
        event.event_type == TraceEventType.INSTANCE_DELETED
        and event.call_id == add_one_call_id
        and any(parameter.name == 'result' and parameter.value == '3'
                for parameter in event.parameters)
        for event in recorder.events()
    )


def test_dynamic_removal_preserves_removed_call_slot():
    """Removing a declared child archives it without retaining its instance."""
    runner = BehaviorTreeRunner()
    recorder = ExecutionTraceRecorder()
    assert CarebtExecutionTraceHooks.install() == ''
    CarebtExecutionTraceHooks.register(runner, recorder)
    try:
        recorder.start_session()
        parent = PendingSequence(runner)
        parent.on_init()
        child_context = parent._child_ec_list[0]
        pending = recorder.snapshot().nodes[-1]
        parent.remove_all_children()
    finally:
        CarebtExecutionTraceHooks.unregister(runner)

    removed = recorder.snapshot().nodes[-1]
    assert pending.class_name == 'AddOne'
    assert pending.lifecycle == TraceLifecycle.PENDING
    assert removed.call_id == pending.call_id
    assert removed.lifecycle == TraceLifecycle.REMOVED
    assert recorder.call_id_for_context(child_context) is None
    assert any(
        event.event_type == TraceEventType.CHILD_REMOVED
        and event.call_id == removed.call_id
        for event in recorder.events()
    )
    assert removed.call_id not in {
        node.call_id for node in recorder.snapshot(include_completed=False).nodes
    }


def test_parameter_values_update_snapshots_without_change_events():
    """Parameter values dirty snapshots without producing high-rate events."""
    runner = BehaviorTreeRunner()
    recorder = ExecutionTraceRecorder()
    recorder.start_session()
    child = AddOne(runner)
    recorder.instance_created(child)
    call_id = recorder._instance_ids[child][0]
    context = type('Context', (), {'instance': child})()
    child._value = 2

    recorder.capture_parameters(context, 0)
    event_count = len(recorder.events())
    assert recorder.take_dirty()
    recorder._nodes[call_id].input_bindings = ('other_binding',)
    recorder.capture_parameters(context, 0)

    assert len(recorder.events()) == event_count
    assert recorder.snapshot().nodes[-1].parameters['0:value'].binding == 'other_binding'
    assert recorder.take_dirty()
    assert not recorder.take_dirty()

    child._value = 3
    recorder.capture_parameters(context, 0)
    assert len(recorder.events()) == event_count
    assert recorder.take_dirty()
    assert recorder.snapshot().nodes[-1].parameters['0:value'].value == '3'


def test_serializer_redacts_nested_secrets_and_truncates_values():
    """Sensitive names are hidden before serialized values are size-limited."""
    serializer = ExecutionTraceSerializer(ExecutionTraceConfig(max_value_bytes=32))

    redacted = serializer.serialize('api_token', 0, '?api_token', 'do-not-publish')
    nested = serializer.serialize(
        'request', 0, '?request', {'password': 'hidden', 'payload': 'x' * 100})

    assert redacted.redacted
    assert 'do-not-publish' not in redacted.value
    assert 'hidden' not in nested.value
    assert nested.truncated
    assert len(nested.value.encode('utf-8')) <= 32


def test_event_handoff_is_bounded_and_reports_drops():
    """A slow publisher receives a marker when the handoff queue overflows."""
    runner = BehaviorTreeRunner()
    recorder = ExecutionTraceRecorder(ExecutionTraceConfig(history_depth=2))
    assert CarebtExecutionTraceHooks.install() == ''
    CarebtExecutionTraceHooks.register(runner, recorder)
    try:
        recorder.start_session()
        parent = PendingSequence(runner)
        parent.on_init()
    finally:
        CarebtExecutionTraceHooks.unregister(runner)

    events = recorder.drain_events()

    assert len(events) == 3
    assert events[-1].event_type == TraceEventType.EVENTS_DROPPED
    assert recorder.snapshot().dropped_event_count > 0


def test_event_handoff_can_be_drained_in_bounded_batches():
    """A bounded drain retains excess events for the next publisher tick."""
    recorder = ExecutionTraceRecorder(ExecutionTraceConfig(history_depth=10))
    recorder.start_session()
    for _ in range(4):
        recorder._emit(TraceEventType.SESSION_ENDED)

    first = recorder.drain_events(3)
    second = recorder.drain_events(3)

    assert len(first) == 3
    assert len(second) == 2
    assert [event.sequence for event in first + second] == [1, 2, 3, 4, 5]


def test_hooks_route_two_runners_to_independent_sessions():
    """Global idempotent wrappers keep runner recorder state isolated."""
    first_runner = BehaviorTreeRunner()
    second_runner = BehaviorTreeRunner()
    first_recorder = ExecutionTraceRecorder()
    second_recorder = ExecutionTraceRecorder()
    assert CarebtExecutionTraceHooks.install() == ''
    assert CarebtExecutionTraceHooks.install() == ''
    CarebtExecutionTraceHooks.register(first_runner, first_recorder)
    CarebtExecutionTraceHooks.register(second_runner, second_recorder)
    try:
        first_recorder.start_session()
        second_recorder.start_session()
        first_runner.run(ParameterSequence)
        second_runner.run(ParameterSequence)
        first_recorder.end_session()
        second_recorder.end_session()
    finally:
        CarebtExecutionTraceHooks.unregister(first_runner)
        CarebtExecutionTraceHooks.unregister(second_runner)

    assert first_recorder.snapshot().session_id != second_recorder.snapshot().session_id
    assert len(first_recorder.snapshot().nodes) == 3
    assert len(second_recorder.snapshot().nodes) == 3
