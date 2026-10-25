"""Unit tests for backends/connect_protocol.py -- the Connect wire format.

No socket, no STK: pure parsing/formatting tests, per the module's design
goal of being testable offline.
"""

import pytest

from stk_bridge.backends import connect_protocol as proto
from stk_bridge.backends.base import StkTime
from stk_bridge.exceptions import ConnectProtocolError


class TestUtcgFormatting:
    def test_round_trip_matches_stk_output_format(self):
        text = "1 Jun 2024 12:34:56.789"
        assert proto.format_utcg(proto.parse_utcg(text)) == text

    def test_single_digit_day_no_leading_zero(self):
        assert proto.format_utcg(proto.parse_utcg("1 Jun 2024 00:00:00.000")) \
            .startswith("1 Jun")

    def test_parse_rejects_garbage(self):
        with pytest.raises(ConnectProtocolError):
            proto.parse_utcg("not a time")

    def test_parse_rejects_bad_month(self):
        with pytest.raises(ConnectProtocolError):
            proto.parse_utcg("1 Xyz 2024 00:00:00.000")

    def test_59_9999_seconds_rounds_up_cleanly(self):
        """Guards the datetime-rejects-second=60 edge case handled in
        parse_utcg."""
        result = proto.parse_utcg("1 Jun 2024 00:00:59.9999995")
        # Should not raise, and should land within 1ms of the next minute.
        next_minute = proto.parse_utcg("1 Jun 2024 00:01:00.000")
        assert abs(result.epoch_seconds - next_minute.epoch_seconds) < 1e-3


class TestQuoting:
    def test_plain_token_unquoted(self):
        assert proto.quote("SSS-1") == "SSS-1"

    def test_whitespace_gets_quoted(self):
        assert proto.quote("1 Jun 2024") == '"1 Jun 2024"'

    def test_empty_string_quoted(self):
        assert proto.quote("") == '""'


class TestBuildCommand:
    def test_float_precision_preserved(self):
        """str() truncates to 12 significant digits; repr() does not.
        The difference is metres of position error over a long
        propagation, so build_command must use repr-grade precision."""
        command = proto.build_command("SetPosition", "*/Satellite/S",
                                      "ICRF", 7000000.123456789)
        assert "7000000.123456789" in command

    def test_bool_becomes_on_off(self):
        assert "On" in proto.build_command("ConControl", True)
        assert "Off" in proto.build_command("ConControl", False)

    def test_time_argument_quoted_utcg(self):
        command = proto.build_command("SetPosition", "*/Satellite/S",
                                      StkTime(1717200000.0))
        assert '"1 Jun 2024' in command


class TestAckNackFraming:
    def test_ack_no_data_completes_immediately(self):
        reader = proto.ConnectResponseReader(expects_data=False)
        reader.feed("ACK\n")
        assert reader.is_complete
        assert reader.result().ack is True

    def test_nack_for_data_command_terminates_without_waiting_for_payload(self):
        """The critical asymmetry: AGI documents that a NACK for a
        data-returning command means no data follows. A reader that
        waited for payload after NACK would hang forever."""
        reader = proto.ConnectResponseReader(expects_data=True)
        reader.feed("NACK object not found\n")
        assert reader.is_complete
        result = reader.result()
        assert result.ack is False
        assert "object not found" in result.nack_text

    def test_single_message_header_then_payload(self):
        reader = proto.ConnectResponseReader(expects_data=True)
        reader.feed("ACK\nGetSTKVersion 10\nSTK v12.6.0\n")
        assert reader.is_complete
        assert "STK v12.6.0" in reader.result().messages

    def test_incremental_feed_across_multiple_calls(self):
        reader = proto.ConnectResponseReader(expects_data=False)
        reader.feed("AC")
        assert not reader.is_complete
        reader.feed("K\n")
        assert reader.is_complete

    def test_force_complete_preserves_partial_buffer(self):
        reader = proto.ConnectResponseReader(expects_data=True)
        reader.feed("ACK\npartial data without newline")
        reader.force_complete()
        assert reader.is_complete
        assert "partial data without newline" in reader.result().messages


class TestReportParsing:
    def test_header_row_then_data_rows(self):
        columns, rows = proto.parse_report_rm([
            "Time, x, y, z",
            "0.0, 1.0, 2.0, 3.0",
            "1.0, 4.0, 5.0, 6.0",
        ])
        assert columns == ["Time", "x", "y", "z"]
        assert len(rows) == 2
        assert rows[0] == ["0.0", "1.0", "2.0", "3.0"]

    def test_empty_messages_yield_no_columns(self):
        columns, rows = proto.parse_report_rm([])
        assert columns == [] and rows == []


class TestLooksLikeDataCommand:
    def test_report_commands_expect_data(self):
        assert proto.looks_like_data_command("Report_RM */Satellite/S ...")

    def test_set_commands_do_not_expect_data(self):
        assert not proto.looks_like_data_command("SetPosition ...")
