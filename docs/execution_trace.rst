CareBT Execution Trace
======================

The execution trace records the dynamic CareBT call tree, node status changes,
contingencies, and declared input and output values. It is read-only and is
disabled by default.

CareBT nodes are represented as call slots and runtime instances. A sequence
can therefore display children which are still pending, while completed and
removed instances remain available for bounded historical inspection.

Enabling tracing
----------------

Set the following parameter on the node derived from ``RosCarebtRunner``::

  execution_trace.enabled: true

The compatibility layer currently supports ``carebt`` version 1.2.4. If the
installed API does not match the expected methods and signatures, tracing is
disabled, an error is logged, and behavior execution continues normally.

Parameters
----------

.. list-table::
   :header-rows: 1

   * - Name
     - Type
     - Default
     - Description
   * - ``execution_trace.enabled``
     - bool
     - false
     - Enables execution tracing.
   * - ``execution_trace.history_depth``
     - int
     - 5000
     - Maximum event handoff and DDS history depth.
   * - ``execution_trace.max_value_bytes``
     - int
     - 1024
     - Maximum serialized size of one parameter value.
   * - ``execution_trace.max_collection_items``
     - int
     - 50
     - Maximum number of items serialized from one collection.
   * - ``execution_trace.max_depth``
     - int
     - 4
     - Maximum recursive serialization depth.
   * - ``execution_trace.snapshot_period_ms``
     - int
     - 250
     - Publication period for dirty snapshots.
   * - ``execution_trace.redacted_name_patterns``
     - string array
     - password, passwd, secret, token, api_key
     - Case-insensitive patterns for parameter names and dictionary keys.

ROS interface
-------------

``/carebt/execution_trace/events``
  Ordered ``carebt_msgs/msg/ExecutionTraceEvent`` deltas. The sequence number,
  rather than the timestamp, defines event order.

``/carebt/execution_trace/snapshot``
  The latest ``carebt_msgs/msg/ExecutionTraceSnapshot``. It allows late-joining
  tools to reconstruct the current tree and recover from an event gap.

Both topics use reliable, transient-local QoS. Event history is bounded by
``execution_trace.history_depth``; the snapshot topic retains one sample.

Only parameters declared through the CareBT input/output syntax are recorded.
Values are converted to bounded JSON. ROS messages are converted through
``rosidl_runtime_py``; unsupported objects use a bounded representation.
Serialization errors, redaction, and truncation never affect tree execution.

RViz panel
----------

Add ``adl_rviz_plugins/CarebtExecutionTracePanel`` in RViz. The panel provides:

* the pending and current call tree with lifecycle and CareBT status;
* an optional removed-node history;
* the latest declared input and output values for the selected node; and
* a bounded, filterable event table.

The pause control stops local rendering only. It does not pause CareBT or ROS
topic publication. The clear control removes local event rows only.

Recording and replay
--------------------

Record the complete trace contract with rosbag2::

  ros2 bag record /carebt/execution_trace/events /carebt/execution_trace/snapshot

Replay the bag before or after opening the RViz panel::

  ros2 bag play <bag-directory>

Bag startup, storage location, rotation, and retention are deployment policy
and are not managed by the tracing feature.