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

"""Runtime tracing for CareBT behavior trees."""

from collections import deque
from copy import deepcopy
from dataclasses import dataclass, field
from enum import IntEnum
from functools import wraps
from importlib.metadata import version
import inspect
import json
import re
from threading import local, RLock
import time
from typing import Any, Callable, Optional
from uuid import uuid4
from weakref import WeakKeyDictionary

from carebt.actionNode import ActionNode
from carebt.controlNode import ControlNode
from carebt.fallbackNode import FallbackNode
from carebt.nodeStatus import NodeStatus
from carebt.parallelNode import ParallelNode
from carebt.rateControlNode import RateControlNode
from carebt.rootNode import RootNode
from carebt.sequenceNode import SequenceNode
from carebt.treeNode import TreeNode
from rosidl_runtime_py.convert import message_to_ordereddict


class TraceEventType(IntEnum):
    """Execution trace event types matching ExecutionTraceEvent.msg."""

    SESSION_STARTED = 0
    SESSION_ENDED = 1
    CHILD_DECLARED = 2
    CHILD_REMOVED = 3
    INSTANCE_ATTACHED = 4
    INSTANCE_DELETED = 5
    STATUS_CHANGED = 6
    CONTINGENCY_CHANGED = 7
    CONTINGENCY_HANDLED = 8
    PARAMETER_CHANGED = 9
    EVENTS_DROPPED = 10


class TraceNodeKind(IntEnum):
    """CareBT node kinds matching ExecutionTraceNode.msg."""

    ACTION = 0
    SEQUENCE = 1
    PARALLEL = 2
    RATE_CONTROL = 3
    FALLBACK = 4
    ROOT = 5
    OTHER = 255


class TraceLifecycle(IntEnum):
    """Runtime call-slot lifecycle matching ExecutionTraceNode.msg."""

    PENDING = 0
    LIVE = 1
    COMPLETED = 2
    REMOVED = 3


@dataclass(frozen=True)
class TraceParameter:
    """One serialized declared CareBT parameter."""

    name: str
    direction: int
    binding: str
    type_name: str
    value: str
    truncated: bool = False
    redacted: bool = False


@dataclass
class TraceNode:
    """Current recorded state of one CareBT call slot."""

    call_id: int
    instance_id: int
    parent_call_id: int
    child_index: int
    class_name: str
    module_name: str
    node_kind: int
    lifecycle: int
    status: int
    contingency_message: str = ''
    parameters: dict[str, TraceParameter] = field(default_factory=dict)
    input_bindings: tuple[str, ...] = ()
    output_bindings: tuple[str, ...] = ()


@dataclass(frozen=True)
class TraceEvent:
    """One immutable execution trace event."""

    stamp_ns: int
    session_id: str
    sequence: int
    event_type: int
    call_id: int = 0
    instance_id: int = 0
    parent_call_id: int = 0
    previous_status: int = 255
    current_status: int = 255
    contingency_message: str = ''
    contingency_handler: str = ''
    parameters: tuple[TraceParameter, ...] = ()


@dataclass(frozen=True)
class TraceSnapshot:
    """Consistent trace state used for late joining and resynchronization."""

    stamp_ns: int
    session_id: str
    last_sequence: int
    session_active: bool
    carebt_version: str
    compatibility_error: str
    dropped_event_count: int
    nodes: tuple[TraceNode, ...]


@dataclass(frozen=True)
class ExecutionTraceConfig:
    """Bounds and publication settings for execution tracing."""

    history_depth: int = 10
    max_value_bytes: int = 1024
    max_collection_items: int = 50
    max_depth: int = 4
    event_publish_period_ms: int = 100
    snapshot_period_ms: int = 250
    redacted_name_patterns: tuple[str, ...] = (
        'password',
        'passwd',
        'secret',
        'token',
        'api_key',
    )


class ExecutionTraceSerializer:
    """Convert arbitrary CareBT parameter values into bounded JSON strings."""

    def __init__(self, config: ExecutionTraceConfig):
        self._config = config
        self._redacted_patterns = tuple(
            re.compile(pattern, re.IGNORECASE)
            for pattern in config.redacted_name_patterns
        )

    def serialize(
        self,
        name: str,
        direction: int,
        binding: str,
        value: Any,
    ) -> TraceParameter:
        """Serialize a named value without allowing failures to escape."""
        type_name = f'{type(value).__module__}.{type(value).__qualname__}'
        if self._is_redacted(name):
            return TraceParameter(
                name, direction, binding, type_name, '"<redacted>"', redacted=True)

        try:
            normalized = self._normalize(value, 0)
            serialized = json.dumps(
                normalized, ensure_ascii=True, sort_keys=True, separators=(',', ':'))
        except Exception as exception:  # noqa: B902 - tracing must not affect execution
            serialized = json.dumps(f'<serialization error: {type(exception).__name__}>')

        encoded = serialized.encode('utf-8')
        truncated = len(encoded) > self._config.max_value_bytes
        if truncated:
            limit = max(0, self._config.max_value_bytes - 3)
            serialized = encoded[:limit].decode('utf-8', errors='ignore') + '...'

        return TraceParameter(
            name, direction, binding, type_name, serialized, truncated=truncated)

    def _normalize(self, value: Any, depth: int) -> Any:
        if depth >= self._config.max_depth:
            return '<maximum depth reached>'
        if value is None or isinstance(value, (bool, int, float, str)):
            return value
        if isinstance(value, bytes):
            return value.hex()
        if isinstance(value, dict):
            result = {}
            items = list(value.items())[:self._config.max_collection_items]
            for key, child_value in items:
                key_string = str(key)
                result[key_string] = (
                    '<redacted>' if self._is_redacted(key_string)
                    else self._normalize(child_value, depth + 1)
                )
            if len(value) > len(items):
                result['<truncated>'] = len(value) - len(items)
            return result
        if isinstance(value, (list, tuple, set)):
            values = list(value)
            result = [
                self._normalize(item, depth + 1)
                for item in values[:self._config.max_collection_items]
            ]
            if len(values) > len(result):
                result.append(f'<{len(values) - len(result)} items truncated>')
            return result
        try:
            return self._normalize(message_to_ordereddict(value), depth + 1)
        except (AttributeError, TypeError, ValueError):
            return repr(value)

    def _is_redacted(self, name: str) -> bool:
        return any(pattern.search(name) for pattern in self._redacted_patterns)


class ExecutionTraceRecorder:
    """Maintain an ordered, bounded model of one CareBT runner."""

    def __init__(
        self,
        config: ExecutionTraceConfig = ExecutionTraceConfig(),
        now_ns: Callable[[], int] = time.time_ns,
    ):
        if config.history_depth < 1:
            raise ValueError('history_depth must be at least one')
        self.config = config
        self._now_ns = now_ns
        self._serializer = ExecutionTraceSerializer(config)
        self._lock = RLock()
        self._events: deque[TraceEvent] = deque(maxlen=config.history_depth)
        self._publish_queue: deque[TraceEvent] = deque(maxlen=config.history_depth)
        self._nodes: dict[int, TraceNode] = {}
        self._context_call_ids: dict[int, int] = {}
        self._instance_ids: WeakKeyDictionary = WeakKeyDictionary()
        self._removed_call_ids: deque[int] = deque()
        self._next_call_id = 1
        self._next_instance_id = 1
        self._sequence = 0
        self._session_id = ''
        self._session_active = False
        self._carebt_version = version('carebt')
        self._compatibility_error = ''
        self._dropped_event_count = 0
        self._unreported_drop_count = 0
        self._dirty = True

    @property
    def compatible(self) -> bool:
        """Return whether instrumentation can be enabled."""
        with self._lock:
            return not self._compatibility_error

    def set_compatibility_error(self, message: str) -> None:
        """Store a compatibility failure for publication in snapshots."""
        with self._lock:
            self._compatibility_error = message
            self._dirty = True

    def start_session(self) -> None:
        """Reset transient state and begin one runner execution."""
        with self._lock:
            self._events.clear()
            self._publish_queue.clear()
            self._nodes.clear()
            self._context_call_ids.clear()
            self._instance_ids = WeakKeyDictionary()
            self._removed_call_ids.clear()
            self._next_call_id = 1
            self._next_instance_id = 1
            self._sequence = 0
            self._dropped_event_count = 0
            self._unreported_drop_count = 0
            self._session_id = str(uuid4())
            self._session_active = True
            self._dirty = True
            self._emit(TraceEventType.SESSION_STARTED)

    def end_session(self) -> None:
        """Finish the current runner execution while preserving its trace."""
        with self._lock:
            if not self._session_active:
                return
            for trace_node in self._nodes.values():
                if trace_node.lifecycle == TraceLifecycle.LIVE:
                    trace_node.lifecycle = TraceLifecycle.COMPLETED
                    self._emit(
                        TraceEventType.INSTANCE_DELETED,
                        call_id=trace_node.call_id,
                        instance_id=trace_node.instance_id,
                        parent_call_id=trace_node.parent_call_id,
                        current_status=trace_node.status,
                        contingency_message=trace_node.contingency_message,
                        parameters=tuple(trace_node.parameters.values()),
                    )
            self._emit(TraceEventType.SESSION_ENDED)
            self._session_active = False
            self._dirty = True

    def instance_created(self, node: TreeNode, call_id: Optional[int] = None) -> None:
        """Register a new runtime node and attach it to a call slot."""
        with self._lock:
            if not self._session_active or node in self._instance_ids:
                return
            if call_id is None or call_id not in self._nodes:
                call_id = self._create_call_slot(node.__class__, 0, 0, '')
            instance_id = self._next_instance_id
            self._next_instance_id += 1
            self._instance_ids[node] = (call_id, instance_id)
            trace_node = self._nodes[call_id]
            trace_node.instance_id = instance_id
            trace_node.class_name = node.__class__.__name__
            trace_node.module_name = node.__class__.__module__
            trace_node.node_kind = self._node_kind(node)
            trace_node.lifecycle = TraceLifecycle.LIVE
            trace_node.status = self._status_value(node.get_status())
            self._emit(
                TraceEventType.INSTANCE_ATTACHED,
                call_id=call_id,
                instance_id=instance_id,
                parent_call_id=trace_node.parent_call_id,
                current_status=trace_node.status,
            )

    def sync_children(
        self,
        parent: ControlNode,
        previous_contexts: tuple[Any, ...],
        params: Optional[str],
    ) -> None:
        """Reconcile a control node's call slots after a child mutation."""
        with self._lock:
            parent_ids = self._instance_ids.get(parent)
            if not self._session_active or parent_ids is None:
                return
            parent_call_id = parent_ids[0]
            current_contexts = tuple(parent._child_ec_list)
            current_identity = {id(context) for context in current_contexts}
            for context in previous_contexts:
                if id(context) not in current_identity:
                    call_id = self._context_call_ids.pop(id(context), None)
                    if call_id is not None:
                        self._mark_removed(call_id)

            previous_identity = {id(context) for context in previous_contexts}
            for index, context in enumerate(current_contexts):
                call_id = self._context_call_ids.get(id(context))
                if call_id is None:
                    binding_spec = params or '' if id(context) not in previous_identity else ''
                    call_id = self._create_call_slot(
                        context.node, parent_call_id, index, binding_spec)
                    self._context_call_ids[id(context)] = call_id
                    self._emit(
                        TraceEventType.CHILD_DECLARED,
                        call_id=call_id,
                        parent_call_id=parent_call_id,
                    )
                self._nodes[call_id].child_index = index

    def call_id_for_context(self, context: Any) -> Optional[int]:
        """Return the call-slot ID associated with an ExecutionContext."""
        with self._lock:
            return self._context_call_ids.get(id(context))

    def status_changed(self, node: TreeNode, previous: NodeStatus, current: NodeStatus) -> None:
        """Record an actual node status transition."""
        with self._lock:
            ids = self._instance_ids.get(node)
            if ids is None or previous == current:
                return
            call_id, instance_id = ids
            trace_node = self._nodes[call_id]
            trace_node.status = self._status_value(current)
            self._emit(
                TraceEventType.STATUS_CHANGED,
                call_id=call_id,
                instance_id=instance_id,
                parent_call_id=trace_node.parent_call_id,
                previous_status=self._status_value(previous),
                current_status=trace_node.status,
            )

    def contingency_changed(self, node: TreeNode, previous: str, current: str) -> None:
        """Record a changed node contingency message."""
        with self._lock:
            ids = self._instance_ids.get(node)
            if ids is None or previous == current:
                return
            call_id, instance_id = ids
            trace_node = self._nodes[call_id]
            trace_node.contingency_message = current
            self._emit(
                TraceEventType.CONTINGENCY_CHANGED,
                call_id=call_id,
                instance_id=instance_id,
                parent_call_id=trace_node.parent_call_id,
                current_status=trace_node.status,
                contingency_message=current,
            )

    def contingency_handled(self, node: TreeNode, entry: Any) -> None:
        """Record a matched contingency immediately before its handler runs."""
        with self._lock:
            ids = self._instance_ids.get(node)
            if ids is None:
                return
            call_id, instance_id = ids
            trace_node = self._nodes[call_id]
            self._emit(
                TraceEventType.CONTINGENCY_HANDLED,
                call_id=call_id,
                instance_id=instance_id,
                parent_call_id=trace_node.parent_call_id,
                current_status=self._status_value(entry.status),
                contingency_message=entry.contingency_message,
                contingency_handler=entry.function,
            )

    def instance_deleted(self, node: TreeNode) -> None:
        """Record completion of a runtime node instance."""
        with self._lock:
            ids = self._instance_ids.get(node)
            if ids is None:
                return
            call_id, instance_id = ids
            trace_node = self._nodes.get(call_id)
            if trace_node is None or trace_node.lifecycle == TraceLifecycle.REMOVED:
                return
            if trace_node.lifecycle != TraceLifecycle.COMPLETED:
                trace_node.lifecycle = TraceLifecycle.COMPLETED
                self._emit(
                    TraceEventType.INSTANCE_DELETED,
                    call_id=call_id,
                    instance_id=instance_id,
                    parent_call_id=trace_node.parent_call_id,
                    current_status=trace_node.status,
                    contingency_message=trace_node.contingency_message,
                    parameters=tuple(trace_node.parameters.values()),
                )

    def capture_parameters(self, child_context: Any, direction: int) -> None:
        """Capture declared child values after a CareBT binding operation."""
        with self._lock:
            child = child_context.instance
            if child is None:
                return
            ids = self._instance_ids.get(child)
            if ids is None:
                return
            call_id, instance_id = ids
            trace_node = self._nodes[call_id]
            if direction == 0:
                names = child._internal_get_in_params()
                bindings = trace_node.input_bindings
            else:
                names = child._internal_get_out_params()
                bindings = trace_node.output_bindings

            parameter_changed = False
            for index, declared_name in enumerate(names):
                name = declared_name.lstrip('?')
                binding = bindings[index] if index < len(bindings) else ''
                value = getattr(child, declared_name.replace('?', '_', 1), None)
                parameter = self._serializer.serialize(name, direction, binding, value)
                key = f'{direction}:{name}'
                previous = trace_node.parameters.get(key)
                trace_node.parameters[key] = parameter
                if previous != parameter:
                    parameter_changed = True
            if parameter_changed:
                self._dirty = True

    def note_error(self, message: str) -> None:
        """Retain the first instrumentation error without raising it."""
        with self._lock:
            if not self._compatibility_error:
                self._compatibility_error = message
                self._dirty = True

    def drain_events(self, maximum: Optional[int] = None) -> tuple[TraceEvent, ...]:
        """Return up to maximum queued events and a drop marker when space permits."""
        with self._lock:
            count = len(self._publish_queue) if maximum is None else min(
                maximum, len(self._publish_queue))
            events = [self._publish_queue.popleft() for _ in range(count)]
            marker_fits = maximum is None or len(events) < maximum
            if not self._publish_queue and self._unreported_drop_count and marker_fits:
                dropped = self._unreported_drop_count
                self._unreported_drop_count = 0
                marker = self._new_event(
                    TraceEventType.EVENTS_DROPPED,
                    contingency_message=f'{dropped} queued events dropped',
                )
                self._events.append(marker)
                events.append(marker)
            return tuple(events)

    def snapshot(self, include_completed: bool = True) -> TraceSnapshot:
        """Return current state, optionally including completed and removed nodes."""
        with self._lock:
            return TraceSnapshot(
                stamp_ns=self._now_ns(),
                session_id=self._session_id,
                last_sequence=self._sequence,
                session_active=self._session_active,
                carebt_version=self._carebt_version,
                compatibility_error=self._compatibility_error,
                dropped_event_count=self._dropped_event_count,
                nodes=tuple(deepcopy([
                    node for node in self._nodes.values()
                    if include_completed or node.lifecycle not in (
                        TraceLifecycle.COMPLETED, TraceLifecycle.REMOVED)
                ])),
            )

    def take_dirty(self) -> bool:
        """Return and clear the state-dirty flag."""
        with self._lock:
            dirty = self._dirty
            self._dirty = False
            return dirty

    def events(self) -> tuple[TraceEvent, ...]:
        """Return the bounded event history for tests and diagnostics."""
        with self._lock:
            return tuple(self._events)

    def _create_call_slot(
        self,
        node_class: type,
        parent_call_id: int,
        child_index: int,
        binding_spec: str,
    ) -> int:
        call_id = self._next_call_id
        self._next_call_id += 1
        input_bindings, output_bindings = self._split_bindings(binding_spec)
        self._nodes[call_id] = TraceNode(
            call_id=call_id,
            instance_id=0,
            parent_call_id=parent_call_id,
            child_index=child_index,
            class_name=node_class.__name__,
            module_name=node_class.__module__,
            node_kind=self._node_kind(node_class),
            lifecycle=TraceLifecycle.PENDING,
            status=255,
            input_bindings=input_bindings,
            output_bindings=output_bindings,
        )
        self._dirty = True
        return call_id

    def _mark_removed(self, call_id: int) -> None:
        trace_node = self._nodes.get(call_id)
        if trace_node is None or trace_node.lifecycle == TraceLifecycle.REMOVED:
            return
        for child in list(self._nodes.values()):
            if child.parent_call_id == call_id:
                self._mark_removed(child.call_id)
        for context_identity, mapped_call_id in list(self._context_call_ids.items()):
            if mapped_call_id == call_id:
                self._context_call_ids.pop(context_identity)
        trace_node.lifecycle = TraceLifecycle.REMOVED
        self._removed_call_ids.append(call_id)
        self._emit(
            TraceEventType.CHILD_REMOVED,
            call_id=call_id,
            instance_id=trace_node.instance_id,
            parent_call_id=trace_node.parent_call_id,
            current_status=trace_node.status,
        )
        while len(self._removed_call_ids) > self.config.history_depth:
            expired_id = self._removed_call_ids.popleft()
            self._nodes.pop(expired_id, None)

    def _emit(self, event_type: TraceEventType, **kwargs: Any) -> None:
        event = self._new_event(event_type, **kwargs)
        self._events.append(event)
        if len(self._publish_queue) == self._publish_queue.maxlen:
            self._publish_queue.popleft()
            self._dropped_event_count += 1
            self._unreported_drop_count += 1
        self._publish_queue.append(event)
        self._dirty = True

    def _new_event(self, event_type: TraceEventType, **kwargs: Any) -> TraceEvent:
        self._sequence += 1
        return TraceEvent(
            stamp_ns=self._now_ns(),
            session_id=self._session_id,
            sequence=self._sequence,
            event_type=event_type,
            **kwargs,
        )

    @staticmethod
    def _status_value(status: Any) -> int:
        return status.value if isinstance(status, NodeStatus) else 255

    @staticmethod
    def _node_kind(node_or_class: Any) -> int:
        node_class = node_or_class if inspect.isclass(node_or_class) else type(node_or_class)
        if issubclass(node_class, RootNode):
            return TraceNodeKind.ROOT
        if issubclass(node_class, RateControlNode):
            return TraceNodeKind.RATE_CONTROL
        if issubclass(node_class, ParallelNode):
            return TraceNodeKind.PARALLEL
        if issubclass(node_class, FallbackNode):
            return TraceNodeKind.FALLBACK
        if issubclass(node_class, SequenceNode):
            return TraceNodeKind.SEQUENCE
        if issubclass(node_class, ActionNode):
            return TraceNodeKind.ACTION
        return TraceNodeKind.OTHER

    @staticmethod
    def _split_bindings(binding_spec: str) -> tuple[tuple[str, ...], tuple[str, ...]]:
        sides = binding_spec.split('=>', maxsplit=1)
        inputs = tuple(filter(None, sides[0].strip().split()))
        outputs = tuple(filter(None, sides[1].strip().split())) if len(sides) == 2 else ()
        return inputs, outputs


class CarebtExecutionTraceHooks:
    """Install version-checked wrappers around CareBT 1.2.4 internals."""

    _supported_versions = {'1.2.4'}
    _lock = RLock()
    _recorders: WeakKeyDictionary = WeakKeyDictionary()
    _originals: dict[tuple[type, str], Callable] = {}
    _installed = False
    _thread_local = local()

    @classmethod
    def install(cls) -> str:
        """Install wrappers once and return an empty string on success."""
        with cls._lock:
            if cls._installed:
                return ''
            compatibility_error = cls._check_compatibility()
            if compatibility_error:
                return compatibility_error

            cls._patch(TreeNode, '__init__', cls._wrap_node_init)
            cls._patch(TreeNode, 'set_status', cls._wrap_status)
            cls._patch(TreeNode, 'set_contingency_message', cls._wrap_contingency)
            cls._patch(
                TreeNode,
                '_internal_append_to_contingency_history',
                cls._wrap_contingency_history,
            )
            cls._patch(TreeNode, '_internal_on_delete', cls._wrap_delete)
            cls._patch(ControlNode, '_internal_bind_in_params', cls._wrap_bind_input)
            cls._patch(ControlNode, '_internal_bind_out_params', cls._wrap_bind_output)

            for node_class, method_name in (
                (SequenceNode, 'append_child'),
                (SequenceNode, 'insert_child_after_current'),
                (SequenceNode, 'remove_all_children'),
                (FallbackNode, 'append_child'),
                (FallbackNode, 'insert_child_after_current'),
                (FallbackNode, 'remove_all_children'),
                (ParallelNode, 'add_child'),
                (ParallelNode, 'remove_child'),
                (ParallelNode, 'remove_all_children'),
                (RateControlNode, 'set_child'),
                (RootNode, 'set_child'),
            ):
                cls._patch(node_class, method_name, cls._wrap_child_mutation)

            for node_class, method_name, context_selector in (
                (SequenceNode, '_internal_create_child_nodes', cls._current_context),
                (FallbackNode, '_internal_create_child_nodes', cls._current_context),
                (ParallelNode, '_internal_create_child_nodes', cls._parallel_new_contexts),
                (RateControlNode, '_internal_create_child_nodes', cls._first_context),
                (RootNode, '_internal_on_tick', cls._first_uninstantiated_context),
            ):
                cls._patch(
                    node_class,
                    method_name,
                    cls._creation_wrapper(context_selector),
                )
            cls._installed = True
            return ''

    @classmethod
    def register(cls, runner: Any, recorder: ExecutionTraceRecorder) -> None:
        """Associate one BehaviorTreeRunner with its recorder."""
        with cls._lock:
            cls._recorders[runner] = recorder

    @classmethod
    def unregister(cls, runner: Any) -> None:
        """Remove a runner association without uninstalling global wrappers."""
        with cls._lock:
            cls._recorders.pop(runner, None)

    @classmethod
    def _check_compatibility(cls) -> str:
        installed_version = version('carebt')
        if installed_version not in cls._supported_versions:
            return (
                f'carebt {installed_version} is unsupported; expected one of '
                f'{sorted(cls._supported_versions)}')
        expected = {
            (TreeNode, '__init__'): ('self', 'bt_runner', 'params'),
            (TreeNode, 'set_status'): ('self', 'node_status'),
            (TreeNode, 'set_contingency_message'): ('self', 'contingency_message'),
            (TreeNode, '_internal_append_to_contingency_history'): ('self', 'entry'),
            (TreeNode, '_internal_on_delete'): ('self',),
            (ControlNode, '_internal_bind_in_params'): ('self', 'child_ec'),
            (ControlNode, '_internal_bind_out_params'): ('self', 'child_ec'),
            (SequenceNode, 'append_child'): ('self', 'node', 'params'),
            (SequenceNode, 'insert_child_after_current'): ('self', 'node', 'params'),
            (SequenceNode, 'remove_all_children'): ('self',),
            (SequenceNode, '_internal_create_child_nodes'): ('self',),
            (FallbackNode, 'append_child'): ('self', 'node', 'params'),
            (FallbackNode, 'insert_child_after_current'): ('self', 'node', 'params'),
            (FallbackNode, 'remove_all_children'): ('self',),
            (FallbackNode, '_internal_create_child_nodes'): ('self',),
            (ParallelNode, 'add_child'): ('self', 'node', 'params'),
            (ParallelNode, 'remove_child'): ('self', 'pos'),
            (ParallelNode, 'remove_all_children'): ('self',),
            (ParallelNode, '_internal_create_child_nodes'): ('self',),
            (RateControlNode, 'set_child'): ('self', 'node', 'params'),
            (RateControlNode, '_internal_create_child_nodes'): ('self',),
            (RootNode, 'set_child'): ('self', 'node', 'params'),
            (RootNode, '_internal_on_tick'): ('self',),
        }
        for (node_class, method_name), parameter_names in expected.items():
            method = getattr(node_class, method_name, None)
            if method is None:
                return f'carebt API mismatch: missing {node_class.__name__}.{method_name}'
            actual_names = tuple(inspect.signature(method).parameters)
            if actual_names != parameter_names:
                return (
                    f'carebt API mismatch: {node_class.__name__}.{method_name}'
                    f'{actual_names}, expected {parameter_names}')
        return ''

    @classmethod
    def _patch(cls, node_class: type, method_name: str, wrapper_factory: Callable) -> None:
        original = getattr(node_class, method_name)
        cls._originals[(node_class, method_name)] = original
        setattr(node_class, method_name, wrapper_factory(original))

    @classmethod
    def _recorder_for_runner(cls, runner: Any) -> Optional[ExecutionTraceRecorder]:
        with cls._lock:
            return cls._recorders.get(runner)

    @classmethod
    def _recorder_for_node(cls, node: TreeNode) -> Optional[ExecutionTraceRecorder]:
        return cls._recorder_for_runner(getattr(node, 'bt_runner', None))

    @staticmethod
    def _notify(recorder: ExecutionTraceRecorder, callback: Callable, *args: Any) -> None:
        try:
            callback(*args)
        except Exception as exception:  # noqa: B902 - tracing must remain observational
            recorder.note_error(
                f'execution trace disabled after {type(exception).__name__}: {exception}')

    @classmethod
    def _wrap_node_init(cls, original: Callable) -> Callable:
        @wraps(original)
        def wrapped(node: TreeNode, bt_runner: Any, params: Optional[str] = None) -> None:
            original(node, bt_runner, params)
            recorder = cls._recorder_for_runner(bt_runner)
            if recorder is not None:
                call_id = cls._take_pending_call_id(bt_runner)
                cls._notify(recorder, recorder.instance_created, node, call_id)
        return wrapped

    @classmethod
    def _wrap_status(cls, original: Callable) -> Callable:
        @wraps(original)
        def wrapped(node: TreeNode, node_status: NodeStatus) -> None:
            previous = node.get_status()
            original(node, node_status)
            recorder = cls._recorder_for_node(node)
            if recorder is not None:
                cls._notify(recorder, recorder.status_changed, node, previous, node_status)
        return wrapped

    @classmethod
    def _wrap_contingency(cls, original: Callable) -> Callable:
        @wraps(original)
        def wrapped(node: TreeNode, contingency_message: str) -> None:
            previous = node.get_contingency_message()
            original(node, contingency_message)
            recorder = cls._recorder_for_node(node)
            if recorder is not None:
                cls._notify(
                    recorder,
                    recorder.contingency_changed,
                    node,
                    previous,
                    contingency_message,
                )
        return wrapped

    @classmethod
    def _wrap_contingency_history(cls, original: Callable) -> Callable:
        @wraps(original)
        def wrapped(node: TreeNode, entry: Any) -> None:
            original(node, entry)
            recorder = cls._recorder_for_node(node)
            if recorder is not None:
                cls._notify(recorder, recorder.contingency_handled, node, entry)
        return wrapped

    @classmethod
    def _wrap_delete(cls, original: Callable) -> Callable:
        @wraps(original)
        def wrapped(node: TreeNode) -> None:
            original(node)
            recorder = cls._recorder_for_node(node)
            if recorder is not None:
                cls._notify(recorder, recorder.instance_deleted, node)
        return wrapped

    @classmethod
    def _wrap_bind_input(cls, original: Callable) -> Callable:
        @wraps(original)
        def wrapped(parent: ControlNode, child_context: Any) -> None:
            original(parent, child_context)
            recorder = cls._recorder_for_node(parent)
            if recorder is not None:
                cls._notify(recorder, recorder.capture_parameters, child_context, 0)
        return wrapped

    @classmethod
    def _wrap_bind_output(cls, original: Callable) -> Callable:
        @wraps(original)
        def wrapped(parent: ControlNode, child_context: Any) -> None:
            original(parent, child_context)
            recorder = cls._recorder_for_node(parent)
            if recorder is not None:
                cls._notify(recorder, recorder.capture_parameters, child_context, 1)
        return wrapped

    @classmethod
    def _wrap_child_mutation(cls, original: Callable) -> Callable:
        @wraps(original)
        def wrapped(parent: ControlNode, *args: Any, **kwargs: Any) -> Any:
            recorder = cls._recorder_for_node(parent)
            previous_contexts = tuple(parent._child_ec_list) if recorder is not None else ()
            result = original(parent, *args, **kwargs)
            if recorder is not None:
                params = kwargs.get('params')
                if params is None and len(args) >= 2:
                    params = args[1]
                cls._notify(
                    recorder,
                    recorder.sync_children,
                    parent,
                    previous_contexts,
                    params,
                )
            return result
        return wrapped

    @classmethod
    def _creation_wrapper(cls, context_selector: Callable) -> Callable:
        def factory(original: Callable) -> Callable:
            @wraps(original)
            def wrapped(parent: ControlNode, *args: Any, **kwargs: Any) -> Any:
                recorder = cls._recorder_for_node(parent)
                if recorder is None:
                    return original(parent, *args, **kwargs)
                call_ids = tuple(
                    call_id
                    for context in context_selector(parent)
                    if (call_id := recorder.call_id_for_context(context)) is not None
                )
                previous = list(getattr(cls._thread_local, 'pending_call_ids', []))
                cls._thread_local.pending_call_ids = previous + [
                    (parent.bt_runner, call_id) for call_id in call_ids]
                try:
                    return original(parent, *args, **kwargs)
                finally:
                    cls._thread_local.pending_call_ids = previous
            return wrapped
        return factory

    @classmethod
    def _take_pending_call_id(cls, runner: Any) -> Optional[int]:
        pending = getattr(cls._thread_local, 'pending_call_ids', [])
        for index, (pending_runner, call_id) in enumerate(pending):
            if pending_runner is runner:
                pending.pop(index)
                return call_id
        return None

    @staticmethod
    def _current_context(parent: ControlNode) -> tuple[Any, ...]:
        if parent._child_ptr < len(parent._child_ec_list):
            context = parent._child_ec_list[parent._child_ptr]
            return (context,) if context.instance is None else ()
        return ()

    @staticmethod
    def _first_context(parent: ControlNode) -> tuple[Any, ...]:
        if parent._child_ec_list and parent._child_ec_list[0].instance is None:
            return (parent._child_ec_list[0],)
        return ()

    @staticmethod
    def _first_uninstantiated_context(parent: ControlNode) -> tuple[Any, ...]:
        return CarebtExecutionTraceHooks._first_context(parent)

    @staticmethod
    def _parallel_new_contexts(parent: ParallelNode) -> tuple[Any, ...]:
        return tuple(parent._child_ec_list[parent._created_child_size:])


class ExecutionTracePublisher:
    """Publish recorder events and snapshots without blocking CareBT hooks."""

    _MAXIMUM_EVENTS_PER_BATCH = 100

    def __init__(self, ros_node: Any, runner: Any, config: ExecutionTraceConfig):
        from carebt_msgs.msg import ExecutionTraceEvent
        from carebt_msgs.msg import ExecutionTraceEventBatch
        from carebt_msgs.msg import ExecutionTraceNode
        from carebt_msgs.msg import ExecutionTraceParameter
        from carebt_msgs.msg import ExecutionTraceSnapshot
        from carebt_msgs.srv import GetExecutionTrace
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
        from rclpy.qos import ReliabilityPolicy

        self._ros_node = ros_node
        self._runner = runner
        self._event_type = ExecutionTraceEvent
        self._event_batch_type = ExecutionTraceEventBatch
        self._node_type = ExecutionTraceNode
        self._parameter_type = ExecutionTraceParameter
        self._snapshot_type = ExecutionTraceSnapshot
        self.recorder = ExecutionTraceRecorder(config)

        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        snapshot_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._event_publisher = ros_node.create_publisher(
            ExecutionTraceEventBatch, '/carebt/execution_trace/events', event_qos)
        self._snapshot_publisher = ros_node.create_publisher(
            ExecutionTraceSnapshot, '/carebt/execution_trace/snapshot', snapshot_qos)
        self._full_tree_service = ros_node.create_service(
            GetExecutionTrace,
            '/carebt/execution_trace/get_full_tree',
            self._get_full_tree,
        )
        self._event_timer = ros_node.create_timer(
            max(1, config.event_publish_period_ms) / 1000.0, self._publish_events)
        self._snapshot_timer = ros_node.create_timer(
            max(1, config.snapshot_period_ms) / 1000.0, self._publish_snapshot)

        compatibility_error = CarebtExecutionTraceHooks.install()
        if compatibility_error:
            self.recorder.set_compatibility_error(compatibility_error)
            ros_node.get_logger().error(
                f'CareBT execution tracing disabled: {compatibility_error}')
        else:
            CarebtExecutionTraceHooks.register(runner, self.recorder)

    @property
    def compatible(self) -> bool:
        """Return whether the installed CareBT API can be traced."""
        return self.recorder.compatible

    def start_session(self) -> None:
        """Begin a trace session when instrumentation is compatible."""
        if self.compatible:
            self.recorder.start_session()

    def end_session(self) -> None:
        """Finish and publish the current trace session."""
        if self.compatible:
            self.recorder.end_session()
        self._publish_events()
        self._publish_snapshot(force=True)

    def destroy(self) -> None:
        """Release runner registration and ROS entities."""
        CarebtExecutionTraceHooks.unregister(self._runner)
        self._ros_node.destroy_timer(self._event_timer)
        self._ros_node.destroy_timer(self._snapshot_timer)
        self._ros_node.destroy_publisher(self._event_publisher)
        self._ros_node.destroy_publisher(self._snapshot_publisher)
        self._ros_node.destroy_service(self._full_tree_service)

    def _publish_events(self) -> None:
        events = self.recorder.drain_events(self._MAXIMUM_EVENTS_PER_BATCH)
        if events:
            message = self._event_batch_type()
            message.events = [self._event_message(event) for event in events]
            self._event_publisher.publish(message)

    def _publish_snapshot(self, force: bool = False) -> None:
        if force or self.recorder.take_dirty():
            snapshot = self.recorder.snapshot(include_completed=False)
            self._snapshot_publisher.publish(self._snapshot_message(snapshot))

    def _get_full_tree(self, request: Any, response: Any) -> Any:
        del request
        response.snapshot = self._snapshot_message(self.recorder.snapshot())
        return response

    def _parameter_message(self, parameter: TraceParameter) -> Any:
        message = self._parameter_type()
        message.name = parameter.name
        message.direction = parameter.direction
        message.binding = parameter.binding
        message.type_name = parameter.type_name
        message.value = parameter.value
        message.truncated = parameter.truncated
        message.redacted = parameter.redacted
        return message

    def _node_message(self, node: TraceNode) -> Any:
        message = self._node_type()
        message.call_id = node.call_id
        message.instance_id = node.instance_id
        message.parent_call_id = node.parent_call_id
        message.child_index = node.child_index
        message.class_name = node.class_name
        message.module_name = node.module_name
        message.node_kind = node.node_kind
        message.lifecycle = node.lifecycle
        message.status = node.status
        message.contingency_message = node.contingency_message
        message.parameters = [
            self._parameter_message(parameter)
            for parameter in node.parameters.values()
        ]
        return message

    def _event_message(self, event: TraceEvent) -> Any:
        message = self._event_type()
        message.stamp = self._time_message(event.stamp_ns)
        message.session_id = event.session_id
        message.sequence = event.sequence
        message.event_type = event.event_type
        message.call_id = event.call_id
        message.instance_id = event.instance_id
        message.parent_call_id = event.parent_call_id
        message.previous_status = event.previous_status
        message.current_status = event.current_status
        message.contingency_message = event.contingency_message
        message.contingency_handler = event.contingency_handler
        message.parameters = [self._parameter_message(value) for value in event.parameters]
        return message

    def _snapshot_message(self, snapshot: TraceSnapshot) -> Any:
        message = self._snapshot_type()
        message.stamp = self._time_message(snapshot.stamp_ns)
        message.session_id = snapshot.session_id
        message.last_sequence = snapshot.last_sequence
        message.session_active = snapshot.session_active
        message.carebt_version = snapshot.carebt_version
        message.compatibility_error = snapshot.compatibility_error
        message.dropped_event_count = snapshot.dropped_event_count
        message.nodes = [self._node_message(node) for node in snapshot.nodes]
        return message

    @staticmethod
    def _time_message(stamp_ns: int) -> Any:
        from builtin_interfaces.msg import Time

        message = Time()
        message.sec = stamp_ns // 1_000_000_000
        message.nanosec = stamp_ns % 1_000_000_000
        return message
