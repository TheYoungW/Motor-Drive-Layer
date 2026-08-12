from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .models import FeedbackReport


class MotorBridgeError(RuntimeError):
    """Base error for the motor-drive-layer Python SDK."""


class AbiLoadError(MotorBridgeError):
    """Raised when libmotor_abi cannot be found or loaded."""


class CallError(MotorBridgeError):
    """Raised when ABI call returns non-zero status."""


class FeedbackRequestError(CallError):
    """Base class for a structured batch-feedback failure."""

    def __init__(self, message: str, report: FeedbackReport) -> None:
        super().__init__(message)
        self.report = report
        self.error_code = report.error_code
        self.missing_motor_ids = report.missing_motor_ids
        self.missing_count = report.missing_count
        self.timeout_ms = report.timeout_ms
        self.expected_count = report.expected_count
        self.received_count = report.received_count


class FeedbackTimeoutError(FeedbackRequestError):
    """No registered motor produced fresh feedback before the deadline."""


class IncompleteFeedbackError(FeedbackRequestError):
    """Only part of the registered motor set produced fresh feedback."""


class FeedbackTransportError(FeedbackRequestError):
    """The feedback request failed at the transport layer."""


class FeedbackMotorFaultError(FeedbackRequestError):
    """The feedback request failed because a motor reported a fault."""
