"""Single-threaded STK worker with a COM-safe apartment.

Why this exists
---------------
COM objects obtained on one thread cannot generally be used from another.
``rclpy``'s ``MultiThreadedExecutor`` dispatches callbacks across a pool,
so without this a service callback and a timer callback would touch the
same STK objects from different threads. The failure mode is not a clean
exception: it is intermittent ``RPC_E_WRONG_THREAD``, silent marshalling
stalls, and occasional process-level hangs that only appear under load.

Everything STK-facing therefore runs on one thread that:

1. calls ``pythoncom.CoInitialize()`` before touching anything COM,
2. owns the backend for its entire lifetime,
3. drains a bounded work queue,
4. calls ``CoUninitialize()`` on the way out.

ROS callbacks submit work and either wait for the result (services) or
fire and forget (streaming commands). This also gives the bridge a single
natural place to apply backpressure, which is what
``BridgeStatus.command_queue_depth`` reports.

The same class is used for the Connect and Mock backends even though they
have no apartment requirement. Uniformity is worth more than the saved
thread: it means there is exactly one concurrency model to reason about,
and swapping backends never changes the threading behaviour.

A deliberate layering exception
-------------------------------
``core/`` is otherwise forbidden from importing COM libraries -- that is
``backends/``'s job. This module is the one exception: it imports
``pythoncom`` to call ``CoInitialize``/``CoUninitialize`` on its own
thread. It cannot be avoided, because apartment initialisation is a
property of the *thread*, not of the object, and this module is what
owns the thread. The import is lazy, guarded by platform and backend
type, and this module never touches an STK object -- it only creates the
apartment that ``backends/`` will use.
"""

from __future__ import annotations

import logging
import platform
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

from stk_bridge.backends.base import BackendType, StkBackend
from stk_bridge.exceptions import BackendUnavailableError, ConnectionLostError

LOGGER = logging.getLogger(__name__)


class QueueFullPolicy:
    """What to do when the work queue is full."""

    DROP_OLDEST = "drop_oldest"
    DROP_NEWEST = "drop_newest"
    BLOCK = "block"
    REJECT = "reject"

    ALL = (DROP_OLDEST, DROP_NEWEST, BLOCK, REJECT)


@dataclass
class _WorkItem:
    function: Callable[[StkBackend], Any]
    description: str
    result: "queue.Queue" = field(default_factory=lambda: queue.Queue(1))
    wants_result: bool = False
    submitted_at: float = field(default_factory=time.monotonic)


class BackendWorker:
    """Runs a backend on a dedicated, COM-initialised thread."""

    def __init__(self, backend: StkBackend, queue_size: int = 256,
                 full_policy: str = QueueFullPolicy.DROP_OLDEST,
                 default_timeout_s: float = 10.0):
        if full_policy not in QueueFullPolicy.ALL:
            raise ValueError(
                f"unknown queue full policy {full_policy!r}; valid values "
                f"are {list(QueueFullPolicy.ALL)}"
            )
        self._backend = backend
        self._queue: "queue.Queue[_WorkItem]" = queue.Queue(queue_size)
        self._queue_size = queue_size
        self._full_policy = full_policy
        self._default_timeout_s = default_timeout_s

        self._thread: Optional[threading.Thread] = None
        self._started = threading.Event()
        self._startup_error: Optional[BaseException] = None
        self._stopping = threading.Event()

        self._dropped = 0
        self._executed = 0
        self._last_latency_ms = 0.0
        self._thread_id = 0

    # -- properties --------------------------------------------------------

    @property
    def backend(self) -> StkBackend:
        """The wrapped backend.

        Exposed for introspection (``backend.info``, ``capabilities``)
        only. Calling STK methods on it directly from another thread is
        exactly the bug this class prevents -- use :meth:`submit`.
        """
        return self._backend

    @property
    def queue_depth(self) -> int:
        return self._queue.qsize()

    @property
    def queue_capacity(self) -> int:
        return self._queue_size

    @property
    def dropped_commands(self) -> int:
        return self._dropped

    @property
    def executed_commands(self) -> int:
        return self._executed

    @property
    def last_latency_ms(self) -> float:
        return self._last_latency_ms

    @property
    def thread_id(self) -> int:
        return self._thread_id

    @property
    def is_running(self) -> bool:
        return (self._thread is not None and self._thread.is_alive()
                and not self._stopping.is_set())

    # -- lifecycle ---------------------------------------------------------

    def start(self, timeout_s: float = 60.0) -> None:
        """Start the worker and block until the backend has connected.

        Blocking here is deliberate: a node that begins publishing before
        STK is attached would emit a burst of empty or default-valued
        telemetry that downstream consumers cannot distinguish from real
        data. Startup failures are re-raised on the calling thread so
        they appear in the launch log rather than only in the worker's.
        """
        if self._thread is not None:
            raise RuntimeError("worker already started")

        self._thread = threading.Thread(
            target=self._run, name="stk-backend-worker", daemon=True
        )
        self._thread.start()

        if not self._started.wait(timeout=timeout_s):
            self._stopping.set()
            raise BackendUnavailableError(
                self._backend.backend_type.value,
                f"backend did not become ready within {timeout_s}s",
            )
        if self._startup_error is not None:
            raise self._startup_error

    def stop(self, timeout_s: float = 10.0) -> None:
        if self._thread is None:
            return
        self._stopping.set()
        # A sentinel wakes the thread even if the queue is empty.
        try:
            self._queue.put_nowait(_WorkItem(lambda b: None, "__stop__"))
        except queue.Full:
            pass
        self._thread.join(timeout=timeout_s)
        if self._thread.is_alive():
            LOGGER.warning("backend worker did not exit within %.1fs; it is "
                           "a daemon thread and will not block shutdown",
                           timeout_s)
        self._thread = None

    def __enter__(self) -> "BackendWorker":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    # -- the worker thread -------------------------------------------------

    def _run(self) -> None:
        com_initialised = self._initialise_com()
        self._thread_id = threading.get_ident()

        try:
            self._backend.connect()
        except BaseException as exc:  # noqa: BLE001 - relay to starter
            self._startup_error = exc
            self._started.set()
            self._finalise_com(com_initialised)
            return

        LOGGER.info("backend worker ready on thread %s (%s)",
                    self._thread_id, self._backend)
        self._started.set()

        try:
            self._drain_loop()
        finally:
            try:
                self._backend.disconnect()
            except Exception:  # pragma: no cover - best effort
                LOGGER.debug("backend disconnect raised", exc_info=True)
            self._finalise_com(com_initialised)

    def _initialise_com(self) -> bool:
        """Initialise a COM apartment if this backend needs one."""
        needs_com = self._backend.backend_type in (
            BackendType.COM, BackendType.ENGINE
        )
        if not needs_com or platform.system() != "Windows":
            return False
        try:
            import pythoncom

            pythoncom.CoInitialize()
            LOGGER.debug("CoInitialize() on worker thread")
            return True
        except ImportError:
            LOGGER.warning(
                "pywin32 not available; COM was not explicitly initialised "
                "on the worker thread. agi.stk12 may initialise it itself, "
                "but install pywin32 to make this deterministic."
            )
        except Exception as exc:  # pragma: no cover
            LOGGER.warning("CoInitialize() failed: %s", exc)
        return False

    def _finalise_com(self, initialised: bool) -> None:
        if not initialised:
            return
        try:
            import pythoncom

            pythoncom.CoUninitialize()
        except Exception:  # pragma: no cover
            LOGGER.debug("CoUninitialize() failed", exc_info=True)

    def _drain_loop(self) -> None:
        while not self._stopping.is_set():
            try:
                item = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item.description == "__stop__":
                break
            self._execute(item)

    def _execute(self, item: _WorkItem) -> None:
        started = time.monotonic()
        try:
            value = item.function(self._backend)
            if item.wants_result:
                item.result.put((True, value))
        except ConnectionLostError as exc:
            LOGGER.error("backend connection lost during %s: %s",
                         item.description, exc)
            if item.wants_result:
                item.result.put((False, exc))
        except BaseException as exc:  # noqa: BLE001 - never kill the worker
            LOGGER.debug("work item %s failed: %s", item.description, exc)
            if item.wants_result:
                item.result.put((False, exc))
        finally:
            self._executed += 1
            self._last_latency_ms = (time.monotonic() - started) * 1000.0

    # -- submission --------------------------------------------------------

    def submit(self, function: Callable[[StkBackend], Any],
               description: str = "work") -> bool:
        """Queue work without waiting for a result.

        Returns False if the item was dropped. Used for the streaming
        command path, where blocking a ROS subscriber callback on STK
        would build unbounded latency into the control loop -- dropping a
        stale sample is strictly better than delivering it late.
        """
        item = _WorkItem(function=function, description=description,
                         wants_result=False)
        return self._enqueue(item)

    def call(self, function: Callable[[StkBackend], Any],
             description: str = "call",
             timeout_s: Optional[float] = None) -> Any:
        """Queue work and block for its result.

        Used by services and by any query whose value is needed now.
        Exceptions raised on the worker thread are re-raised here, with
        their original type preserved so callers can catch
        ``QueryError`` / ``CommandError`` as normal.
        """
        if not self.is_running:
            raise BackendUnavailableError(
                self._backend.backend_type.value, "worker is not running"
            )

        item = _WorkItem(function=function, description=description,
                         wants_result=True)
        if not self._enqueue(item, force_block=True):
            raise BackendUnavailableError(
                self._backend.backend_type.value,
                f"could not queue {description!r}: work queue full",
            )

        timeout = timeout_s if timeout_s is not None else self._default_timeout_s
        try:
            ok, value = item.result.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError(
                f"{description!r} did not complete within {timeout}s. The "
                f"worker may be blocked on a slow STK operation; check "
                f"queue_depth ({self.queue_depth}/{self.queue_capacity})."
            ) from None

        if not ok:
            raise value
        return value

    def _enqueue(self, item: _WorkItem, force_block: bool = False) -> bool:
        policy = QueueFullPolicy.BLOCK if force_block else self._full_policy
        try:
            if policy == QueueFullPolicy.BLOCK:
                self._queue.put(item, timeout=self._default_timeout_s)
            else:
                self._queue.put_nowait(item)
            return True
        except queue.Full:
            pass

        if policy == QueueFullPolicy.DROP_OLDEST:
            # Drop the head so the freshest sample survives. For a
            # control loop, the newest command is the only one that
            # matters; the queued older ones are already stale.
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(item)
                self._dropped += 1
                return True
            except (queue.Empty, queue.Full):
                pass

        self._dropped += 1
        if self._dropped in (1, 10, 100) or self._dropped % 1000 == 0:
            LOGGER.warning(
                "work queue full (%d/%d), policy=%s, dropped=%d. STK is not "
                "keeping up with the configured rate.",
                self.queue_depth, self._queue_size, self._full_policy,
                self._dropped,
            )
        return False

    def statistics(self) -> dict:
        return {
            "queue_depth": self.queue_depth,
            "queue_capacity": self.queue_capacity,
            "executed": self._executed,
            "dropped": self._dropped,
            "last_latency_ms": self._last_latency_ms,
            "thread_id": self._thread_id,
            "running": self.is_running,
        }
