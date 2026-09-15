"""Synthetic export/vector tests; never access Chrome or real credentials."""
import base64
import unittest
from unittest.mock import patch
from urllib.parse import urlencode

import cscs_export_otp as otp


def vint(value):
    result = bytearray()
    while value > 127:
        result.append((value & 127) | 128)
        value >>= 7
    result.append(value)
    return bytes(result)


def field(number, value):
    if isinstance(value, int):
        return vint(number << 3) + vint(value)
    return vint((number << 3) | 2) + vint(len(value)) + value


def account(name=b'lterenzi', issuer=b'CSCS', algorithm=1, digits=1, kind=2):
    # RFC 6238 public test-vector key, not a user's secret.
    return b''.join(field(k, v) for k, v in [
        (1, b'12345678901234567890'), (2, name), (3, issuer),
        (4, algorithm), (5, digits), (6, kind),
    ])


def export(*accounts, batch_size=1):
    data = b''.join(field(1, value) for value in accounts) + field(3, batch_size)
    return 'otpauth-migration://offline?' + urlencode({'data': base64.b64encode(data).decode()})


class ExportTests(unittest.TestCase):
    def test_exact_selection_and_known_totp_vector(self):
        secret = otp.select_secret(export(account(issuer=b'Other'), account(), account(name=b'other')))
        self.assertEqual(otp.totp(secret, 59), '287082')
        self.assertEqual(otp.totp(secret, 1111111109), '081804')

    def test_prefixed_username(self):
        secret = otp.select_secret(export(account(name=b'CSCS:lterenzi')))
        self.assertEqual(otp.totp(secret, 59), '287082')

    def test_wrong_and_duplicate_accounts_fail(self):
        for uri in [export(account(name=b'other')), export(account(issuer=b'Other')),
                    export(account(), account())]:
            with self.subTest(uri_kind='synthetic'), self.assertRaises(ValueError):
                otp.select_secret(uri)

    def test_unsupported_parameters_and_incomplete_batch_fail(self):
        for uri in [export(account(algorithm=2)), export(account(digits=2)),
                    export(account(kind=1)), export(account(), batch_size=2)]:
            with self.subTest(uri_kind='synthetic'), self.assertRaises(ValueError):
                otp.select_secret(uri)

    def test_malformed_protobuf_fails(self):
        for value in [b'\x80', b'\x0a\x05x', b'\x00\x01']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                list(otp.fields(value))

    def test_only_cscs_login_origin_is_accepted(self):
        otp.assert_cscs_url('https://auth.cscs.ch/auth/realms/cscs/login-actions/authenticate?x=y')
        for url in ['http://auth.cscs.ch/auth/realms/cscs/login-actions/authenticate',
                    'https://auth.cscs.ch.evil.test/auth/realms/cscs/login-actions/authenticate',
                    'https://auth.cscs.ch:8443/auth/realms/cscs/login-actions/authenticate',
                    'https://aai-logon.ethz.ch/', 'https://auth.cscs.ch/',
                    'https://user@auth.cscs.ch/auth/realms/cscs/login-actions/authenticate']:
            with self.subTest(url=url), self.assertRaises(ValueError):
                otp.assert_cscs_url(url)

    def test_wrong_foreground_stops_before_typing(self):
        with patch.object(otp, 'run', return_value=b'456') as run:
            with self.assertRaises(ValueError):
                otp.submit_desktop(b'synthetic', '123', (1, 2), (3, 4))
            run.assert_called_once_with('xdotool', 'getactivewindow')

    def test_wrong_origin_stops_before_generating_or_typing(self):
        calls = []
        def fake_run(*args, input=None):
            calls.append(args)
            if args == ('xdotool', 'getactivewindow'):
                return b'123'
            if args == ('xclip', '-o', '-selection', 'clipboard'):
                return b'https://example.com/'
            return b''
        with patch.object(otp, 'run', side_effect=fake_run), patch.object(otp, 'fresh_code') as code:
            with self.assertRaises(ValueError):
                otp.submit_desktop(b'synthetic', '123', (1, 2), (3, 4))
            code.assert_not_called()
        self.assertFalse(any(args[:2] == ('xdotool', 'type') for args in calls))


if __name__ == '__main__':
    unittest.main()
