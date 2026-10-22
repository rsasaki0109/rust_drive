"""The observed transient mirror failure must not change the selected inputs."""
import contextlib
import io
import unittest
from unittest.mock import patch
import urllib.error

import camera_linear_fetch as fetch


class FixedInputDownload(unittest.TestCase):
    def test_transient_503_retries_the_identical_url_and_keeps_read_bound(self):
        url = 'https://example.invalid/pinned-commit/image.jpg'
        error = urllib.error.HTTPError(url, 503, 'unavailable', {}, None)
        with patch.object(fetch.urllib.request, 'urlopen', side_effect=[error, io.BytesIO(b'abcdef')]) as open_url, \
                patch.object(fetch.time, 'sleep'), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(fetch.download(url, 2), b'abc')
        self.assertEqual(open_url.call_args_list[0], open_url.call_args_list[1])

    def test_403_is_not_retried(self):
        url = 'https://example.invalid/pinned-commit/image.jpg'
        error = urllib.error.HTTPError(url, 403, 'forbidden', {}, None)
        with patch.object(fetch.urllib.request, 'urlopen', side_effect=error) as open_url:
            with self.assertRaises(urllib.error.HTTPError):
                fetch.download(url, 2)
        self.assertEqual(open_url.call_count, 1)

    def test_transient_retries_are_bounded(self):
        url = 'https://example.invalid/pinned-commit/image.jpg'
        error = urllib.error.HTTPError(url, 503, 'unavailable', {}, None)
        with patch.object(fetch.urllib.request, 'urlopen', side_effect=error) as open_url, \
                patch.object(fetch.time, 'sleep') as sleep, contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(urllib.error.HTTPError):
                fetch.download(url, 2)
        self.assertEqual((open_url.call_count, sleep.call_count), (3, 2))


if __name__ == '__main__':
    unittest.main()
