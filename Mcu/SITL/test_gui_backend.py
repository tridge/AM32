"""Background EEPROM polling must never wait on, or retain, an ESC tab."""
import threading
import time
import unittest
from unittest.mock import Mock

from sitl_gui_backend import EepromPoller


class EepromPollerTests(unittest.TestCase):
    def test_one_read_at_a_time_and_result_delivery(self):
        entered, release = threading.Event(), threading.Event()
        expected = (b'EEPROM', {'kv': 900, 'poles': 14})
        def fetch():
            entered.set()
            if not release.wait(2):
                raise TimeoutError('test did not release fetch')
            return expected
        client = Mock(fetch=Mock(side_effect=fetch))
        poll = EepromPoller(client)
        try:
            self.assertTrue(poll.request())
            self.assertTrue(entered.wait(1))
            for _ in range(20):
                self.assertFalse(poll.request())
                self.assertIsNone(poll.take())
            self.assertEqual(client.fetch.call_count, 1)
            release.set()
            result = None
            deadline = time.monotonic() + 2
            while result is None and time.monotonic() < deadline:
                result = poll.take()
                time.sleep(0.005)
            self.assertEqual(result, expected)
            self.assertFalse(poll.pending)
        finally:
            release.set()
            poll.close()

    def test_closed_tab_does_not_wait_or_start_more_reads(self):
        entered, release, finished = (threading.Event() for _ in range(3))
        def fetch():
            entered.set()
            release.wait(2)
            finished.set()
            return (None, None)
        poll = EepromPoller(Mock(fetch=Mock(side_effect=fetch)))
        try:
            poll.request()
            self.assertTrue(entered.wait(1))
            poll.close()
            self.assertFalse(finished.is_set())  # close did not join the read
            self.assertFalse(poll.request())
        finally:
            release.set()
            self.assertTrue(finished.wait(1))


if __name__ == '__main__':
    unittest.main()
